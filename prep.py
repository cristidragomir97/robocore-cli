#!/usr/bin/env python3
import os
import sys
import yaml
from python_on_whales import DockerClient
from utils.common import load_config, get_hosts, render_template
from typing import Optional

def prep_main(project_root: str, component: Optional[str] = None):
    # 1) cd into project so load_config() finds config.yaml
    os.chdir(project_root)
    cfg   = load_config()
    hosts = get_hosts(cfg)

    comp_root = os.path.abspath(cfg['components_dir'])

    # 2) single template for each component
    script_dir = os.path.dirname(os.path.realpath(__file__))
    tpl_dir    = os.path.join(script_dir, 'templates')
    df_tpl     = os.path.join(tpl_dir, 'Dockerfile.j2')
    compose_tpl= os.path.join(tpl_dir, 'docker-compose.j2')

    if not os.path.isfile(df_tpl):
        raise FileNotFoundError(f"Dockerfile template not found: {df_tpl}")
    if not os.path.isfile(compose_tpl):
        raise FileNotFoundError(f"Compose template not found: {compose_tpl}")

    docker      = DockerClient()
    tag         = cfg.get('tag', 'latest')
    deploy_mode = cfg.get('deploy_mode', 'image').lower()

    # choose components
    all_comps = sorted(os.listdir(comp_root))
    comps     = [component] if component else all_comps

    built_images = []
    for name in comps:
        comp_dir = os.path.join(comp_root, name)
        ros_ws   = os.path.join(comp_dir, 'ros_ws')
        meta_f   = os.path.join(comp_dir, 'component.yaml')
        if not (os.path.isdir(ros_ws) and os.path.isfile(meta_f)):
            print(f"[prep] skipping '{name}' (missing ros_ws/ or component.yaml)")
            continue

        meta = yaml.safe_load(open(meta_f))
        meta['name'] = name
        pre  = meta.get('preinstall', [])
        post = meta.get('postinstall', [])

        image_tag = f"{cfg['registry']}/{cfg['image_prefix']}_{name}:{tag}"

        # 3) render Dockerfile
        out_df = os.path.join(comp_dir, 'Dockerfile')
        render_template(
            df_tpl, out_df,
            base_image    = cfg['base_image'],
            ros_distro    = cfg['ros_distro'],
            preinstall    = pre,
        )

        # 4) build & push or just build locally
        # platforms = linux/arch for each host
        archs     = sorted({h.arch for h in hosts})
        platforms = [f"linux/{a}" for a in archs]

        docker.buildx.build(
                comp_dir,
                file=out_df,
                platforms=platforms,
                tags=[image_tag],
                push=True
        )

        built_images.append({**meta, 'image': image_tag})

    # 5) pull on each host (only in image mode really matters)
    port = cfg.get('docker_port', 2375)
    for host in hosts:
        url    = f"tcp://{host.ip}:{port}"
        remote = DockerClient(host=url)
        for comp in built_images:
            img = comp['image']
            print(f"[prep] {host.name}: pulling '{img}'")
            remote.pull(img)

    # 6) render docker-compose.yml
    render_template(
        compose_tpl,
        cfg['compose_file'],
        components    = built_images,
        registry      = cfg['registry'],
        image_prefix  = cfg['image_prefix'],
        ros_distro    = cfg['ros_distro'],
        ros_domain_id = cfg['ros_domain_id'],
        mount_root    = cfg['mount_root'],
        tag           = tag
    )
    print(f"[prep] Wrote '{cfg['compose_file']}'")


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) > 2:
        print("Usage: prep.py [project_root] [component]")
        sys.exit(1)
    project_root = args[0] if len(args) >= 1 else "."
    component    = args[1] if len(args) == 2 else None
    prep_main(project_root, component)
