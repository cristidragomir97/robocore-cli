#!/usr/bin/env python3
import os, yaml
from python_on_whales import DockerClient
from utils.common import load_config, get_hosts, render_template

def main():
    cfg         = load_config()
    hosts       = get_hosts(cfg)
    comp_root   = cfg['components_dir']
    tpl_dir     = os.path.abspath(os.path.join(os.path.dirname(__file__),'..','templates'))
    builder_tpl = os.path.join(tpl_dir, 'Dockerfile.builder.j2')
    runtime_tpl = os.path.join(tpl_dir, 'Dockerfile.runtime.j2')
    compose_tpl = os.path.join(tpl_dir, 'docker-compose.j2')

    # Validate templates
    for path in (builder_tpl, runtime_tpl, compose_tpl):
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Template not found: {path}")

    local_docker = DockerClient()
    runtime_images = []
    tag = cfg.get('tag','latest')

    for name in sorted(os.listdir(comp_root)):
        comp_dir = os.path.join(comp_root, name)
        ros_ws   = os.path.join(comp_dir, 'ros_ws')
        meta_f   = os.path.join(comp_dir, 'component.yaml')
        if not (os.path.isdir(ros_ws) and os.path.isfile(meta_f)):
            continue

        meta = yaml.safe_load(open(meta_f))
        meta['name'] = name

        builder_tag = f"{cfg['registry']}/{cfg['image_prefix']}_{name}:builder-{tag}"
        runtime_tag = f"{cfg['registry']}/{cfg['image_prefix']}_{name}:{tag}"

        # 1) Builder image
        if meta.get('build_context', False):
            out_b = os.path.join(comp_dir, 'Dockerfile.builder')
            render_template(builder_tpl, out_b,
                            base_image=cfg['base_image'],
                            ros_distro=cfg['ros_distro'])
            print(f"[prep] Building builder image '{builder_tag}'")
            local_docker.build(
                comp_dir,
                file=out_b,
                tags=[builder_tag]
            )
            print(f"[prep] Pushing builder image '{builder_tag}'")
            local_docker.push(builder_tag)

        # 2) Runtime image
        out_r = os.path.join(comp_dir, 'Dockerfile.runtime')
        render_template(runtime_tpl, out_r,
                        base_image=builder_tag,
                        ros_distro=cfg['ros_distro'])
        print(f"[prep] Building & pushing runtime multi-arch image '{runtime_tag}'")
        local_docker.buildx.build(
            comp_dir,
            file=out_r,
            platforms=["linux/amd64"],
            tags=[runtime_tag],
            push=True
        )

        runtime_images.append({**meta, 'image': runtime_tag})

        port = cfg.get('docker_port', 2375)
        for host in hosts:
            base_url = f"tcp://{host.ip}:{port}"
            print(f"[prep] Connecting to Docker on {host.name} at {base_url}")
            remote = DockerClient(host=base_url)
            for comp in runtime_images:
                img = comp['image']
                print(f"[prep] {host.name}: pulling '{img}'")
                remote.pull(img)

        # ─── 4) Generate docker-compose.yml ──────────────────────────────────
        compose_out = cfg['compose_file']
        render_template(
            compose_tpl, compose_out,
            components     = runtime_images,
            registry       = cfg['registry'],
            image_prefix   = cfg['image_prefix'],
            ros_distro     = cfg['ros_distro'],
            ros_domain_id  = cfg['ros_domain_id'],
            tag            = tag
        )
        print(f"[prep] Wrote '{compose_out}' with updated image tags.")

if __name__=='__main__':
    main()
