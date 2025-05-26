#!/usr/bin/env python3
import os
import sys
import yaml
from typing import List, Optional

from core.config   import Config
from core.renderer import TemplateRenderer
from core.docker   import DockerHelper
from core.models   import Component, Host

def detect_component_features(comp: Component, cfg: Config) -> dict:
    """
    Return a dict of Boolean flags describing which inputs this component uses.
    """
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

def build_base_image(cfg: Config, renderer: TemplateRenderer, docker: DockerHelper):
    """Render & build the shared base image (ROS2 + common + apt)."""
    ubuntu_map = {"humble": "jammy", "galactic": "jammy", "foxy": "focal"}
    ubuntu     = ubuntu_map.get(cfg.ros_distro, "jammy")
    base_df    = os.path.join(".", "Dockerfile.base")

    renderer.render_base(
        out_path     = base_df,
        ros_distro   = cfg.ros_distro,
        ubuntu       = ubuntu,
        common_pkgs  = cfg.common_packages,
        apt_packages = cfg.apt_packages
    )
    if not os.path.isfile(base_df):
        sys.exit(f"[stage] ERROR: {base_df} missing after render")

    base_tag = f"{cfg.registry}/{cfg.image_prefix}_base:{cfg.ros_distro}-{cfg.tag}"
    print(f"[stage] Building base image → {base_tag}")
    docker.build_multiarch(
        image_tag  = base_tag,
        context    = ".",
        dockerfile = "Dockerfile.base",
        platforms  = cfg.platforms,
        push       = True
    )
    return base_tag

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
                          docker: DockerHelper) -> dict:
    """Render & build one component, returning its compose entry dict."""
    # 1) Pick host & arch
    host = get_host(hosts_map, comp, cfg)
  

    comp_dir   = os.path.abspath(comp.folder)
    feats      = detect_component_features(comp, cfg)

    # 2) If VCS, emit repos.yaml
    repos_file = os.path.join(comp_dir, "vcs_ws", "repos.yaml")
    if feats["has_repos"]:
        os.makedirs(os.path.dirname(repos_file), exist_ok=True)
        repo_map = {
            r.folder: {"type":"git","url":r.url,"version":r.version}
            for r in comp.repositories
        }
        with open(repos_file, "w") as f:
            yaml.safe_dump({"repositories":repo_map},
                           f, default_flow_style=False, sort_keys=False)

    # 3) Local source path (for Jinja)
    src_path = os.path.join(comp_dir, "ros_ws", "src")
    comp_src = os.path.relpath(src_path, ".") if feats["comp_src_exists"] else ""

    # 4) Render Dockerfile.j2
    comp_df = os.path.join(comp_dir, "Dockerfile")
    renderer.render(
        "Dockerfile.j2",
        comp_df,
        base_image         = base_tag,
        ros_distro         = cfg.ros_distro,
        ros_domain_id      = cfg.ros_domain_id,
        comp               = comp,
        comp_src           = comp_src,
        postinstall        = comp.postinstall or [],
        repos_file         = os.path.relpath(repos_file, ".") if feats["has_repos"] else "",
        **feats,   # gives common_ws_exists, has_repos, comp_src_exists, apt_exists
        apt_packages       = comp.apt_packages or []
    )
    if not os.path.isfile(comp_df):
        sys.exit(f"[stage] ERROR: {comp_df} missing after render")

    # 5) Build single-arch image
    img_tag  = comp.image_tag(cfg)
    platform = f"linux/{host.arch}"
    print(f"[stage] Building '{comp.name}' → {img_tag} for [{platform}]")
    docker.build_multiarch(
        image_tag  = img_tag,
        context    = ".",
        dockerfile = os.path.relpath(comp_df, "."),
        platforms  = [platform],
        push       = True,
 
    )

    return {
        "name":        comp.name,
        "image":       img_tag,
        "devices":     comp.devices,
        "ports":       comp.ports,
        "entrypoint":  comp.entrypoint,
        "launch_args": comp.launch_args,
        "has_repos":   feats["has_repos"],
        "comp_src":    feats["comp_src_exists"]}
    

def render_compose(staged: List[dict], cfg: Config, renderer: TemplateRenderer):
    """Emit the final docker‐compose.yml."""
    out = os.path.join(".", cfg.compose_file)
    renderer.render_compose(out, staged, cfg)
    print(f"[stage] Wrote '{cfg.compose_file}'")

def stage_main(project_root: str, component: Optional[str] = None):
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    cfg       = Config.load(project_root)
    hosts_map = {h.name: h for h in cfg.hosts}
    comps     = cfg.filter_components(name=component)
    if not comps:
        sys.exit(f"[stage] ERROR: no components to stage (filter={component})")

    # Templates live next to this script
    tpl_dir  = os.path.abspath(os.path.join(
                  os.path.dirname(__file__), '..', 'templates'))
    renderer = TemplateRenderer(tpl_dir)
    docker   = DockerHelper()

    # Stage 1: base image
    base_tag = build_base_image(cfg, renderer, docker)

    # Stage 2: each component
    staged = []
    for comp in comps:
        entry = build_component_image(
            comp, cfg, hosts_map, base_tag, renderer, docker
        )
        
        docker.pull_image_on_host(get_host(hosts_map, comp, cfg), base_tag)
        staged.append(entry)

    # Stage 3: compose
    render_compose(staged, cfg, renderer)



if __name__ == "__main__":
    pr, comp = (sys.argv[1], sys.argv[2]) if len(sys.argv) == 3 else (".", None)
    stage_main(pr, comp)
