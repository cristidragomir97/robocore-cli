#!/usr/bin/env python3
import os
import sys
import yaml
from collections import defaultdict
from typing import List, Optional

from core.config   import Config
from core.renderer import TemplateRenderer
from core.docker   import DockerHelper
from core.models   import Component, Host

def setup_managed_workspace(comp: Component, cfg: Config) -> str:
    """Create managed workspace and symlink sources"""
    workspace_dir = os.path.join(cfg.root, comp.managed_workspace)
    src_dir = os.path.join(workspace_dir, "src")

    # Create workspace directories
    os.makedirs(src_dir, exist_ok=True)

    # Clean existing symlinks
    for item in os.listdir(src_dir):
        item_path = os.path.join(src_dir, item)
        if os.path.islink(item_path):
            os.unlink(item_path)

    # Link source packages
    source_paths = comp.get_source_paths()
    has_sources = False

    for source_path in source_paths:
        abs_source = os.path.join(cfg.root, source_path)
        if os.path.exists(abs_source):
            if os.path.isdir(abs_source):
                # Check if this is a ROS package or workspace src directory
                if os.path.exists(os.path.join(abs_source, "package.xml")):
                    # It's a ROS package, link it directly
                    package_name = os.path.basename(abs_source)
                    link_target = os.path.join(src_dir, package_name)
                    if os.path.exists(link_target):
                        os.unlink(link_target)
                    os.symlink(abs_source, link_target)
                    print(f"  - linked package: {source_path} -> {comp.name}/src/{package_name}")
                    has_sources = True
                elif os.path.exists(os.path.join(abs_source, "src")):
                    # It's a workspace src directory, link all packages within it
                    src_contents = os.path.join(abs_source, "src")
                    if os.path.exists(src_contents):
                        for pkg in os.listdir(src_contents):
                            pkg_path = os.path.join(src_contents, pkg)
                            if os.path.isdir(pkg_path) and os.path.exists(os.path.join(pkg_path, "package.xml")):
                                link_target = os.path.join(src_dir, pkg)
                                if os.path.exists(link_target):
                                    os.unlink(link_target)
                                os.symlink(pkg_path, link_target)
                                print(f"  - linked package: {source_path}/src/{pkg} -> {comp.name}/src/{pkg}")
                                has_sources = True
                else:
                    print(f"  - WARNING: {source_path} doesn't appear to be a ROS package or workspace")
            else:
                print(f"  - WARNING: Source path {source_path} is not a directory")
        else:
            print(f"  - WARNING: Source path {source_path} does not exist")

    return workspace_dir if has_sources else ""

def detect_component_features(comp: Component, cfg: Config) -> dict:
    has_common = bool(cfg.common_packages)
    has_repos  = bool(comp.repositories)
    has_apt    = bool(comp.apt_packages)

    # Set up managed workspace and detect local sources
    workspace_dir = setup_managed_workspace(comp, cfg)
    has_local = bool(workspace_dir)

    # For legacy components using folder field
    if comp.folder and not workspace_dir:
        src_path = os.path.join(comp.folder, cfg.workspace_dir, "src")
        has_local = os.path.isdir(src_path)

    print(f"[stage] Component '{comp.name}' features:")
    print(f"  - common_ws_exists: {has_common}")
    print(f"  - has_repos:        {has_repos}")
    print(f"  - comp_src_exists:  {has_local}")
    print(f"  - apt_exists:       {has_apt}")
    if workspace_dir:
        print(f"  - managed_workspace: {workspace_dir}")

    return {
        "common_ws_exists": has_common,
        "has_repos":        has_repos,
        "comp_src_exists":  has_local,
        "apt_exists":       has_apt,
        "workspace_dir":    workspace_dir
    }

def get_host(hosts_map: dict, comp: Component, cfg: Config) -> Host:
    if not comp.runs_on:
        sys.exit(f"[stage] ERROR: component '{comp.name}' missing 'runs_on'")
    host: Host = hosts_map.get(comp.runs_on)
    if not host:
        sys.exit(f"[stage] ERROR: runs_on '{comp.runs_on}' not defined")
    return host

def build_component_image(comp: Component,
                          cfg: Config,
                          hosts_map: dict,
                          base_tag: str,
                          renderer: TemplateRenderer,
                          docker: DockerHelper,
                          participant_id: int) -> dict:

    host = get_host(hosts_map, comp, cfg)
    feats = detect_component_features(comp, cfg)

    # Determine working directory for files (Dockerfile, repos.yaml, etc.)
    if feats["workspace_dir"]:
        # Use managed workspace
        comp_dir = os.path.abspath(feats["workspace_dir"])
    elif comp.folder:
        # Legacy support
        comp_dir = os.path.abspath(comp.folder)
    else:
        # Create minimal workspace for repos-only components
        comp_dir = os.path.join(cfg.root, ".robocore", "workspaces", comp.name)
        os.makedirs(comp_dir, exist_ok=True)

    # Find DDS manager host (default to first host if none specified)
    dds_manager = next((h for h in cfg.hosts if getattr(h, "manager", False)), None)
    if not dds_manager:
        if len(cfg.hosts) > 0:
            dds_manager = cfg.hosts[0]
            print(f"[stage] No host marked as manager, using '{dds_manager.name}' as DDS manager")
        else:
            sys.exit("[stage] ERROR: No hosts defined in config.")

    # Handle VCS repositories
    repos_file = os.path.join(comp_dir, "repos.yaml")
    if feats["has_repos"]:
        repo_map = {
            r.folder: {"type": "git", "url": r.url, "version": r.version}
            for r in comp.repositories
        }
        with open(repos_file, "w") as f:
            yaml.safe_dump({"repositories": repo_map}, f, default_flow_style=False, sort_keys=False)

    # Determine source paths for Docker build
    if feats["comp_src_exists"]:
        if feats["workspace_dir"]:
            # Use managed workspace src directory
            comp_src_rel = os.path.relpath(os.path.join(comp_dir, "src"), cfg.root)
        else:
            # Legacy path
            comp_src_rel = os.path.relpath(os.path.join(comp_dir, cfg.workspace_dir, "src"), cfg.root)
    else:
        comp_src_rel = ""

    repos_file_rel = os.path.relpath(repos_file, cfg.root) if feats["has_repos"] else ""
    superclient_path = os.path.join(comp_dir, "superclient.xml")
    superclient_rel = os.path.relpath(superclient_path, cfg.root)

    renderer.render_superclient(
        out_path=superclient_path,
        component_name=comp.name,
        participantID=participant_id,
        this_host_ip=host.ip,
        dds_server_host_ip=dds_manager.ip
    )

    dockerfile_path = os.path.join(comp_dir, "Dockerfile")
    renderer.render_dockerfile(
        out_path=dockerfile_path,
        base_image=base_tag,
        ros_distro=cfg.ros_distro,
        ros_domain_id=cfg.ros_domain_id,
        comp=comp,
        comp_src=comp_src_rel,
        repos_file=repos_file_rel,
        superclient_path=superclient_rel,
        apt_packages=comp.apt_packages or [],
        pip_packages=comp.pip_packages or [],
        postinstall=comp.postinstall or [],
        has_repos=feats["has_repos"],
        comp_src_exists=feats["comp_src_exists"],
        enable_apt_caching=cfg.enable_apt_caching,
        dds_server_ip=dds_manager.ip
    )

    if not os.path.isfile(dockerfile_path):
        sys.exit(f"[stage] ERROR: {dockerfile_path} missing after render")

    img_tag = comp.image_tag(cfg)
    platform = f"linux/{host.arch}"
    print(f"[stage] Building '{comp.name}' → {img_tag} for [{platform}]")

    # Use absolute paths for context and dockerfile
    docker.build_multiarch(
        image_tag=img_tag,
        context=cfg.root,
        dockerfile=dockerfile_path,
        platforms=[platform],
        push=True
    )

    return {
        "name": comp.name,
        "image": img_tag,
        "devices": comp.devices,
        "ports": comp.ports,
        "entrypoint": comp.entrypoint,
        "launch_args": comp.launch_args,
        "has_repos": feats["has_repos"],
        "comp_src": feats["comp_src_exists"]
    }

def stage_main(project_root: str,
               component: Optional[str] = None,
               refresh: bool = False):
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    cfg = Config.load(project_root)
    hosts_map = {h.name: h for h in cfg.hosts}
    comps = cfg.filter_components(name=component)
    if not comps:
        sys.exit(f"[stage] ERROR: no components to stage (filter={component})")

    tpl_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'templates'))
    renderer = TemplateRenderer(tpl_dir)
    docker = DockerHelper()

    # Group components per host
    components_by_host = defaultdict(list)
    for comp in comps:
        components_by_host[comp.runs_on].append(comp)

    # Build base image once for all platforms
    from commands.prepare_base import prepare_base_main
    prepare_base_main(project_root)
    base_tag = cfg.base_image

    staged_by_host = {}

    for host_name, host_comps in components_by_host.items():
        host = hosts_map[host_name]
        staged = []

        for j, comp in enumerate(host_comps):
            participant_id = j + 2  # Start from 2
            entry = build_component_image(
                comp, cfg, hosts_map, base_tag, renderer, docker,
                participant_id=participant_id
            )
            docker.pull_image_on_host(host, base_tag)
            staged.append(entry)

        staged_by_host[host_name] = staged

    # Write one docker-compose.<host>.yaml per host
    for host_name, staged_comps in staged_by_host.items():
        compose_name = f"docker-compose.{host_name}.yaml"
        out_path = os.path.join(".", compose_name)
        renderer.render_compose(out_path, staged_comps, cfg)
        print(f"[stage] Wrote '{compose_name}'")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--project-root", default=".", help="Path to project root")
    parser.add_argument("-c", "--component", default=None, help="Component to restage")
    parser.add_argument("--refresh", action="store_true", help="Only regenerate docker-compose.yml")
    args = parser.parse_args()
    stage_main(args.project_root, args.component, args.refresh)
