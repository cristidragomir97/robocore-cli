#!/usr/bin/env python3
import os
import sys
import yaml
from collections import defaultdict
from typing import List, Optional

from colorama import Fore
from core.config   import Config
from core.renderer import TemplateRenderer
from core.docker   import DockerHelper
from core.models   import Component, Host
from python_on_whales.exceptions import DockerException

def resolve_source_packages(comp: Component, cfg: Config) -> List[tuple]:
    """
    Resolve source paths into a list of (relative_path, package_name) tuples.
    These paths are relative to cfg.root and will be used directly in Docker COPY commands.
    """
    packages = []
    source_paths = comp.get_source_paths()

    for source_path in source_paths:
        abs_source = os.path.join(cfg.root, source_path)
        if not os.path.exists(abs_source):
            print(f"  - WARNING: Source path {source_path} does not exist")
            continue

        if not os.path.isdir(abs_source):
            print(f"  - WARNING: Source path {source_path} is not a directory")
            continue

        # Check if this is a ROS package
        if os.path.exists(os.path.join(abs_source, "package.xml")):
            package_name = os.path.basename(abs_source)
            packages.append((source_path, package_name))
            print(f"  - found package: {source_path} -> {package_name}")
        # Check if this is a workspace src directory containing packages
        elif os.path.exists(os.path.join(abs_source, "src")):
            src_contents = os.path.join(abs_source, "src")
            for pkg in os.listdir(src_contents):
                pkg_path = os.path.join(src_contents, pkg)
                if os.path.isdir(pkg_path) and os.path.exists(os.path.join(pkg_path, "package.xml")):
                    rel_pkg_path = os.path.join(source_path, "src", pkg)
                    packages.append((rel_pkg_path, pkg))
                    print(f"  - found package: {rel_pkg_path} -> {pkg}")
        else:
            print(f"  - WARNING: {source_path} doesn't appear to be a ROS package or workspace")

    return packages

def detect_component_features(comp: Component, cfg: Config) -> dict:
    has_common = bool(cfg.common_packages)
    has_repos  = bool(comp.repositories)
    has_apt    = bool(comp.apt_packages)

    # Resolve source packages (no copying, just detection)
    source_packages = resolve_source_packages(comp, cfg)
    has_local = bool(source_packages)

    print(f"[stage] Component '{comp.name}' features:")
    print(f"  - common_ws_exists: {has_common}")
    print(f"  - has_repos:        {has_repos}")
    print(f"  - comp_src_exists:  {has_local}")
    print(f"  - apt_exists:       {has_apt}")
    if source_packages:
        print(f"  - source_packages:  {len(source_packages)} packages")

    return {
        "common_ws_exists": has_common,
        "has_repos":        has_repos,
        "comp_src_exists":  has_local,
        "apt_exists":       has_apt,
        "source_packages":  source_packages
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
    img_tag = comp.image_tag(cfg)
    platform = f"linux/{host.arch}"

    # Handle external pre-built images
    if comp.image:
        print(f"[stage] Using external image '{comp.name}' → {img_tag}")
        # No build needed, just use the external image
        return {
            "name": comp.name,
            "image": img_tag,
            "devices": comp.devices,
            "ports": comp.ports,
            "environment": comp.environment,
            "entrypoint": comp.entrypoint,
            "launch_args": comp.launch_args,
            "has_repos": False,
            "comp_src": False,
            "shm_size": comp.shm_size,
            "ipc_mode": comp.ipc_mode,
            "cpuset": comp.cpuset,
            "mount_shm": comp.mount_shm,
            "rt_enabled": comp.rt_enabled,
            "privileged": comp.privileged,
            "runtime": comp.runtime,
            "gpu_count": comp.gpu_count,
            "gpu_device_ids": comp.gpu_device_ids,
        }

    # Handle custom Dockerfile builds
    if comp.build:
        build_context = os.path.join(cfg.root, comp.build)
        dockerfile = os.path.join(build_context, "Dockerfile")

        if not os.path.exists(dockerfile):
            sys.exit(f"[stage] ERROR: Dockerfile not found at {dockerfile}")

        print(f"[stage] Building '{comp.name}' from custom Dockerfile → {img_tag} for [{platform}]")

        docker.build_multiarch(
            image_tag=img_tag,
            context=build_context,
            dockerfile=dockerfile,
            platforms=[platform],
            push=True
        )

        return {
            "name": comp.name,
            "image": img_tag,
            "devices": comp.devices,
            "ports": comp.ports,
            "environment": comp.environment,
            "entrypoint": comp.entrypoint,
            "launch_args": comp.launch_args,
            "has_repos": False,
            "comp_src": False,
            "shm_size": comp.shm_size,
            "ipc_mode": comp.ipc_mode,
            "cpuset": comp.cpuset,
            "mount_shm": comp.mount_shm,
            "rt_enabled": comp.rt_enabled,
            "privileged": comp.privileged,
            "runtime": comp.runtime,
            "gpu_count": comp.gpu_count,
            "gpu_device_ids": comp.gpu_device_ids,
        }

    # Robocore-managed build (current logic)
    feats = detect_component_features(comp, cfg)

    # Working directory for generated files (Dockerfile, repos.yaml, superclient.xml)
    comp_dir = os.path.join(cfg.root, ".robocore", "workspaces", comp.name)
    os.makedirs(comp_dir, exist_ok=True)

    # Find DDS manager host (default to first host if none specified)
    dds_manager = next((h for h in cfg.hosts if h.manager), None)
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

    repos_file_rel = os.path.relpath(repos_file, cfg.root) if feats["has_repos"] else ""
    superclient_path = os.path.join(comp_dir, "superclient.xml")
    superclient_rel = os.path.relpath(superclient_path, cfg.root)

    renderer.render_superclient(
        out_path=superclient_path,
        participantID=participant_id,
        this_host_ip=host.effective_dds_ip,
        dds_server_host_ip=dds_manager.effective_dds_ip
    )

    dockerfile_path = os.path.join(comp_dir, "Dockerfile")
    renderer.render_dockerfile(
        out_path=dockerfile_path,
        base_image=base_tag,
        ros_distro=cfg.ros_distro,
        ros_domain_id=cfg.ros_domain_id,
        comp=comp,
        source_packages=feats["source_packages"],
        repos_file=repos_file_rel,
        superclient_path=superclient_rel,
        apt_packages=comp.apt_packages or [],
        pip_packages=comp.pip_packages or [],
        postinstall=comp.postinstall or [],
        has_repos=feats["has_repos"],
        comp_src_exists=feats["comp_src_exists"],
        enable_apt_caching=cfg.enable_apt_caching,
        dds_server_ip=dds_manager.effective_dds_ip
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
        "environment": comp.environment,
        "entrypoint": comp.entrypoint,
        "launch_args": comp.launch_args,
        "has_repos": feats["has_repos"],
        "comp_src": feats["comp_src_exists"],
        "shm_size": comp.shm_size,
        "ipc_mode": comp.ipc_mode,
        "cpuset": comp.cpuset,
        "mount_shm": comp.mount_shm,
        "rt_enabled": comp.rt_enabled,
        "privileged": comp.privileged,
        "runtime": comp.runtime,
        "gpu_count": comp.gpu_count,
        "gpu_device_ids": comp.gpu_device_ids,
    }

def stage_main(project_root: str,
               component: Optional[str] = None,
               refresh: bool = False,
               force_base: bool = False):
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
    prepare_base_main(project_root, force=force_base)
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
            try:
                docker.pull_image_on_host(host, base_tag)
            except DockerException:
                print(Fore.RED + f"[stage] Failed to pull base image on host '{host.name}' ({host.ip})", file=sys.stderr)
                raise
            staged.append(entry)

        staged_by_host[host_name] = staged

    # Find DDS manager host for compose files
    dds_manager = next((h for h in cfg.hosts if h.manager), None)
    if not dds_manager and len(cfg.hosts) > 0:
        dds_manager = cfg.hosts[0]

    # Write one docker-compose.<host>.yaml per host
    for host_name, staged_comps in staged_by_host.items():
        host = hosts_map[host_name]
        compose_name = f"docker-compose.{host_name}.yaml"
        out_path = os.path.join(".", compose_name)
        # Only include dds_server section in the manager host's compose file
        host_dds_manager = dds_manager if host.manager else None
        renderer.render_compose(out_path, staged_comps, cfg, host=host, dds_manager=host_dds_manager)
        print(f"[stage] Wrote '{compose_name}'")

    # Generate superclient.xml for local development (robostack)
    if dds_manager:
        robocore_dir = os.path.join(project_root, '.robocore')
        os.makedirs(robocore_dir, exist_ok=True)
        superclient_path = os.path.join(robocore_dir, 'superclient.xml')

        # Get local IP for DDS communication
        import socket
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((dds_manager.effective_dds_ip, 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            local_ip = "127.0.0.1"

        renderer.render_superclient(
            superclient_path,
            participantID=1,  # Local dev machine uses participant ID 1
            this_host_ip=local_ip,
            dds_server_host_ip=dds_manager.effective_dds_ip
        )
        print(Fore.GREEN + f"[stage] ✓ Generated superclient.xml for local development")
        print(Fore.CYAN + f"[stage]   Location: {superclient_path}")
        print(Fore.CYAN + f"[stage]   DDS Server: {dds_manager.effective_dds_ip}:11811")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--project-root", default=".", help="Path to project root")
    parser.add_argument("-c", "--component", default=None, help="Component to restage")
    parser.add_argument("--refresh", action="store_true", help="Only regenerate docker-compose.yml")
    parser.add_argument("--force-base", action="store_true", help="Force rebuild of base image")
    args = parser.parse_args()
    stage_main(args.project_root, args.component, args.refresh, args.force_base)
