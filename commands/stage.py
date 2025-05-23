#!/usr/bin/env python3
import os
import sys
import yaml
from python_on_whales import DockerClient
from utils.common import (
    load_config, get_hosts, get_components, render_template
)
from typing import Optional

def stage_main(project_root: str, component: Optional[str] = None):
    # 1) CD into project root
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    # 2) Load config & hosts & components
    cfg       = load_config()
    hosts     = get_hosts(cfg)
    all_comps = get_components(cfg)
    common_pkgs = cfg.get("common_packages", [])

    # 3) Filter if needed
    if component:
        all_comps = [c for c in all_comps if c.name == component]
        if not all_comps:
            raise ValueError(f"No component named '{component}' in config.yaml")

    # 4) Templates
    script_dir  = os.path.dirname(os.path.realpath(__file__))
    tpl_dir     = os.path.join(script_dir, "..", "templates")
    df_tpl      = os.path.join(tpl_dir, "Dockerfile.j2")
    compose_tpl = os.path.join(tpl_dir, "docker-compose.j2")
    for path in (df_tpl, compose_tpl):
        if not os.path.isfile(path):
            sys.exit(f"[stage] ERROR: template not found: {path}")

    docker = DockerClient()
    tag    = cfg.get("tag", "latest")

    staged = []
    for comp in all_comps:
        comp_dir = os.path.abspath(comp.folder)
        src_dir  = os.path.join(comp_dir, "ros_ws", "src")
        if not os.path.isdir(src_dir):
            print(f"[stage] skipping '{comp.name}' (no ros_ws/src)")
            continue

        # 5) Dump per-component repos.yaml if needed
        repos = getattr(comp, "repositories", [])
        repos_file = os.path.join(comp_dir, "ros_ws", "repos.yaml")
        if repos:
            os.makedirs(os.path.dirname(repos_file), exist_ok=True)
            with open(repos_file, "w") as f:
                yaml.safe_dump({"repositories": repos}, f)

        # 6) Compute comp_src and relative paths
        comp_src = os.path.relpath(src_dir, project_root)
        rel_repos = os.path.relpath(repos_file, project_root) if repos else ""

        # 7) Render the multi-stage Dockerfile
        out_df = os.path.join(comp_dir, "Dockerfile")
        render_template(
            df_tpl, out_df,
            ros_distro    = cfg["ros_distro"],
            ros_domain_id = cfg["ros_domain_id"],
            extra_apt     = cfg.get("extra_apt", []),
            common_pkgs   = common_pkgs,
            comp          = comp,
            comp_src      = comp_src,
            has_repos     = bool(repos),
            repos_file    = rel_repos
        )

        # 8) Build & push multi-arch
        rel_df    = os.path.relpath(out_df, project_root)
        archs     = sorted({h.arch for h in hosts})
        platforms = [f"linux/{a}" for a in archs]
        image_tag = f"{cfg['registry']}/{cfg['image_prefix']}_{comp.name}:{tag}"

        print(f"[stage] Building '{comp.name}' → {image_tag} on {platforms}")
        docker.buildx.build(
            context = project_root,
            file    = rel_df,
            platforms=platforms,
            tags    =[image_tag],
            push    = True
        )

        staged.append({
            "name": comp.name,
            "image": image_tag,
            **{k: getattr(comp, k, []) for k in ("devices","ports","entrypoint","launch_args")}
        })

    # 9) Render compose with all staged images
    render_template(
        compose_tpl,
        cfg["compose_file"],
        components    = staged,
        mount_root    = cfg["mount_root"],
        ros_distro    = cfg["ros_distro"],
        ros_domain_id = cfg["ros_domain_id"]
    )
    print(f"[stage] Wrote '{cfg['compose_file']}'")


if __name__ == "__main__":
    pr, comp = (sys.argv[1], sys.argv[2]) if len(sys.argv)==3 else (".", None)
    stage_main(pr, comp)
