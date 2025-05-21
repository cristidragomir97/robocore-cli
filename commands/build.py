#!/usr/bin/env python3
import os
import sys
import shutil
from python_on_whales import DockerClient
from utils.common import load_config, get_components
from typing import Optional

def build_main(project_root: str, component: Optional[str] = None):
    os.chdir(project_root)
    cfg    = load_config()
    docker = DockerClient()

    build_root = os.path.abspath(cfg['build_dir'])
    os.makedirs(build_root, exist_ok=True)

    all_comps = get_components(cfg)
    if component:
        all_comps = [c for c in all_comps if c.name == component]
        if not all_comps:
            raise ValueError(f"No component '{component}' in config.yaml")

    tag = cfg.get('tag','latest')
    for comp in all_comps:
        comp_ws = os.path.join(build_root, comp.name, 'ros_ws')
        # clean
        if os.path.isdir(comp_ws):
            print(f"[build] Cleaning old workspace at {comp_ws}")
            shutil.rmtree(comp_ws)
        os.makedirs(os.path.join(comp_ws,'src'), exist_ok=True)

        # copy source
        src_src = os.path.join(comp.folder, 'ros_ws', 'src')
        print(f"[build] Copying source → {comp_ws}/src")
        shutil.copytree(src_src, os.path.join(comp_ws,'src'), dirs_exist_ok=True)

        # assemble build command
        steps = [
            "source /opt/ros/$ROS_DISTRO/setup.bash",
            "colcon build --symlink-install --install-base install"
        ]
        for post in comp.postinstall or []:
            steps.append(
                "source /opt/ros/$ROS_DISTRO/setup.bash && "
                "source /ros_ws/install/setup.bash && "
                f"{post}"
            )
        full_cmd = " && ".join(steps)
        print(f"[build] [{comp.name}] {full_cmd}")

        # run builder image (built by prep)
        image = f"{cfg['registry']}/{cfg['image_prefix']}_{comp.name}:{tag}"
        docker.run(
            image=image,
            command=["bash","-lc", full_cmd],
            remove=True,
            tty=True,
            workdir="/ros_ws",
            envs={"ROS_DISTRO": cfg['ros_distro']},
            volumes=[(os.path.abspath(comp_ws), "/ros_ws", "rw")]
        )
        print(f"[build] '{comp.name}' done; install at {comp_ws}/install")

if __name__=="__main__":
    args = sys.argv[1:]
    if len(args)>2:
        print("Usage: builder.py [project_root] [component]")
        sys.exit(1)
    pr   = args[0] if len(args)>=1 else "."
    comp = args[1] if len(args)==2 else None
    build_main(pr, comp)
