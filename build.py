#!/usr/bin/env python3
import os
import sys
import shutil
import yaml
from python_on_whales import DockerClient
from utils.common import load_config
from typing import Optional

def build_main(project_root: str, component: Optional[str] = None):
    # 1) cd into project so config.yaml resolves
    os.chdir(project_root)
    cfg    = load_config()
    docker = DockerClient()

    comp_root  = os.path.abspath(cfg['components_dir'])
    build_root = os.path.abspath(cfg['build_dir'])
    tag        = cfg.get('tag', 'latest')

    os.makedirs(build_root, exist_ok=True)

    all_comps = sorted(os.listdir(comp_root))
    if component:
        if component not in all_comps:
            raise ValueError(f"Component '{component}' not found in {comp_root}")
        comps = [component]
    else:
        comps = all_comps

    for name in comps:
        comp_dir = os.path.join(comp_root, name)
        src_dir  = os.path.join(comp_dir, 'ros_ws', 'src')
        meta_f   = os.path.join(comp_dir, 'component.yaml')

        if not (os.path.isdir(src_dir) and os.path.isfile(meta_f)):
            print(f"[build] skipping '{name}' (no ros_ws/src/ or component.yaml)")
            continue

        # load any post-install hooks
        meta = yaml.safe_load(open(meta_f))
        post = meta.get("post_install", [])

        # 2) prepare a clean workspace root
        ws_root = os.path.join(build_root, name, 'ros_ws')
        if os.path.isdir(ws_root):
            print(f"[build] Cleaning old workspace at {ws_root}")
            shutil.rmtree(ws_root)
        os.makedirs(os.path.join(ws_root, 'src'), exist_ok=True)

        # copy in your sources
        print(f"[build] Copying source for '{name}' → {ws_root}/src")
        shutil.copytree(src_dir, os.path.join(ws_root, 'src'), dirs_exist_ok=True)

        # 3) build + post_install command
        cmds = [
            # source ROS
            "source /opt/ros/$ROS_DISTRO/setup.bash",
            # build into /ros_ws/install
            "colcon build --symlink-install --install-base install"
        ]
        # wrap each post_install with proper sourcing
        if post:
            post_cmds = post if isinstance(post, list) else [post]
            for p in post_cmds:
                cmds.append(
                    "source /opt/ros/$ROS_DISTRO/setup.bash && "
                    "source /ros_ws/install/setup.bash && "
                    f"{p}"
                )

        full_cmd = " && ".join(cmds)
        print(f"[build] Running builder for '{name}': {full_cmd}")

        # 4) run the single unified image (built by prep) as builder
        image = f"{cfg['registry']}/{cfg['image_prefix']}_{name}:{tag}"
        docker.run(
            image=image,
            command=["bash", "-lc", full_cmd],
            remove=True,
            tty=True,
            workdir="/ros_ws",
            envs={"ROS_DISTRO": cfg['ros_distro']},
            # mount the whole workspace
            volumes=[(os.path.abspath(ws_root), "/ros_ws", "rw")],
        )

        print(f"[build] '{name}' built; workspace is at {ws_root}/install")

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) > 2:
        print("Usage: builder.py [project_root] [component]")
        sys.exit(1)
    project_root = args[0] if len(args) >= 1 else "."
    component    = args[1] if len(args) == 2 else None
    build_main(project_root, component)
