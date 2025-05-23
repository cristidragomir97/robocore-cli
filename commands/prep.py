#!/usr/bin/env python3
import os
import sys
from python_on_whales import DockerClient
from utils.common import (
    load_config,
    get_hosts,
    get_components,
    render_template
)
from typing import Optional

def prep_main(project_root: str, component: Optional[str] = None):
    # 1) chdir into project root so all paths are relative to it
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    cfg       = load_config()
    hosts     = get_hosts(cfg)
    all_comps = get_components(cfg)
    common    = cfg.get("common_packages", [])

    # 2) filter if user asked for a single component
    if component:
        all_comps = [c for c in all_comps if c.name == component]
        if not all_comps:
            raise ValueError(f"No component named '{component}' in config.yaml")

    # 3) locate templates
    script_dir  = os.path.dirname(os.path.realpath(__file__))
    tpl_dir     = os.path.join(script_dir, "..", "templates")
    df_tpl      = os.path.join(tpl_dir, "Dockerfile.j2")
    compose_tpl = os.path.join(tpl_dir, "docker-compose.j2")
    for t in (df_tpl, compose_tpl):
        if not os.path.isfile(t):
            raise FileNotFoundError(f"Template not found: {t}")

    docker      = DockerClient()
    tag         = cfg.get("tag", "latest")
    deploy_mode = cfg.get("deploy_mode", "image").lower()

    built_images = []
    for comp in all_comps:
        comp_dir = os.path.abspath(comp.folder)
        ros_ws   = os.path.join(comp_dir, "ros_ws")
        if not os.path.isdir(ros_ws):
            print(f"[prep] skipping '{comp.name}' (no ros_ws/ found)")
            continue

        image_tag = f"{cfg['registry']}/{cfg['image_prefix']}_{comp.name}:{tag}"

        # 4) compute comp_src relative to project_root
        comp_src = os.path.relpath(
            os.path.join(comp_dir, "ros_ws", "src"),
            project_root
        )

        # 5) render Dockerfile with common_pkgs + preinstall hooks
        out_df = os.path.join(comp_dir, "Dockerfile")
        render_template(
            df_tpl, out_df,
            comp         = comp,
            comp_src     = comp_src,
            base_image   = cfg["base_image"],
            ros_distro   = cfg["ros_distro"],
            common_pkgs  = common,
            preinstall   = comp.preinstall or []
        )

        # 6) buildx.build using project_root as context
        #    and Dockerfile path relative to project_root
        rel_df    = os.path.relpath(out_df, project_root)
        archs     = sorted({h.arch for h in hosts})
        platforms = [f"linux/{a}" for a in archs]

        print(f"[prep] Building & pushing multi-arch image '{image_tag}' for {platforms}")
        docker.buildx.build(
            project_root,    # context: project root
            file=rel_df,     # Dockerfile relative to context
            platforms=platforms,
            tags=[image_tag],
            push=True
        )

        built_images.append({
            "name"        : comp.name,
            "image"       : image_tag,
            "devices"     : comp.devices or [],
            "ports"       : comp.ports or [],
            "postinstall" : comp.postinstall or [],
            "entrypoint"  : comp.entrypoint,
            "launch_args" : comp.launch_args
        })

    # 7) pull each runtime image on every host
    port = cfg.get("docker_port", 2375)
    for host in hosts:
        url    = f"tcp://{host.ip}:{port}"
        remote = DockerClient(host=url)
        for img in built_images:
            print(f"[prep] {host.name}: pulling '{img['image']}'")
            remote.pull(img["image"])

    # 8) render docker-compose.yml
    render_template(
        compose_tpl,
        cfg["compose_file"],
        components    = built_images,
        mount_root    = cfg["mount_root"],
        ros_distro    = cfg["ros_distro"],
        ros_domain_id = cfg["ros_domain_id"]
    )
    print(f"[prep] Wrote '{cfg['compose_file']}'")


if __name__ == "__main__":
    pr, comp = (sys.argv[1], sys.argv[2]) if len(sys.argv) == 3 else (".", None)
    prep_main(pr, comp)
