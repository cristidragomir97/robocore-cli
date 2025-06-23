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

def detect_component_features(comp: Component, cfg: Config) -> dict:
    has_common = bool(cfg.common_packages)
    has_repos  = bool(comp.repositories)
    src_path   = os.path.join(comp.folder, "ros_ws", "src")
    has_local  = os.path.isdir(src_path)
    has_apt    = bool(comp.apt_packages)

    print(f"[stage] Component '{comp.name}' features:")
    print(f"  - common_ws_exists: {has_common}")
    print(f"  - has_repos:        {has_repos}")
    print(f"  - comp_src_exists:  {has_local}")
    print(f"  - apt_exists:       {has_apt}")

    return {
        "common_ws_exists": has_common,
        "has_repos":        has_repos,
        "comp_src_exists":  has_local,
        "apt_exists":       has_apt
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
    comp_dir = os.path.abspath(comp.folder)
    feats = detect_component_features(comp, cfg)

    dds_manager = next((h for h in cfg.hosts if getattr(h, "manager", False)), None)
    if not dds_manager:
        sys.exit("[stage] ERROR: No host marked as manager for DDS.")

    repos_file = os.path.join(comp_dir, "vcs_ws", "repos.yaml")
    if feats["has_repos"]:
        os.makedirs(os.path.dirname(repos_file), exist_ok=True)
        repo_map = {
            r.folder: {"type": "git", "url": r.url, "version": r.version}
            for r in comp.repositories
        }
        with open(repos_file, "w") as f:
            yaml.safe_dump({"repositories": repo_map}, f, default_flow_style=False, sort_keys=False)

    comp_src_rel = os.path.relpath(os.path.join(comp_dir, "ros_ws", "src"), cfg.root) if feats["comp_src_exists"] else ""
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

    docker.build_multiarch(
        image_tag=img_tag,
        context=".",
        dockerfile=os.path.relpath(dockerfile_path, "."),
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
