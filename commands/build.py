#!/usr/bin/env python3
import os
import sys
import shutil
import yaml
from typing import Optional

from python_on_whales import DockerClient
from core.config   import Config
from core.models   import Component, Host

def build_main(project_root: str, component: Optional[str] = None):
    """
    For each component that has a local ros_ws/src folder:
      1) Copy src → build/<comp>/ros_ws/src
      2) Run any postinstall hooks
      3) Invoke the builder image (already staged) to colcon build → install
    """
    # 1) chdir into project
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    # 2) load config & pick components
    cfg   = Config.load(project_root)
    docker = DockerClient()
    comps = cfg.filter_components(name=component)
    if not comps:
        print(f"[build] No components to build (filter={component})")
        return

    build_root = os.path.abspath(cfg.build_dir)
    os.makedirs(build_root, exist_ok=True)

    # 3) loop
    for comp in comps:
        comp_name = comp.name

        # Determine source directory (new managed workspace or legacy)
        if comp.folder:
            # Legacy support
            comp_dir = os.path.abspath(comp.folder)
            src_dir = os.path.join(comp_dir, cfg.workspace_dir, 'src')
        else:
            # Use managed workspace
            managed_workspace = os.path.join(project_root, comp.managed_workspace)
            src_dir = os.path.join(managed_workspace, 'src')

        # only build if there's local source
        if not os.path.exists(src_dir) or not os.listdir(src_dir):
            print(f"[build] Skipping '{comp_name}' (no source packages found)")
            continue

        # prepare a clean workspace
        ws_root = os.path.join(build_root, comp_name, cfg.workspace_dir)

        # copy in your sources
        print(f"[build] Copying source for '{comp_name}' → {ws_root}/src")
        if os.path.exists(os.path.join(ws_root, 'src')):
            shutil.rmtree(os.path.join(ws_root, 'src'))

        # Handle symlinks properly - copy the actual content, not the symlinks
        shutil.copytree(src_dir, os.path.join(ws_root, 'src'),
                       dirs_exist_ok=True, symlinks=False, copy_function=shutil.copy2)

        # load any post-install hooks
        # (your Component model should expose postinstall as a List[str])
        post_hooks = comp.postinstall or []

        # assemble the build command
        cmds = [
            "source /opt/ros/$ROS_DISTRO/setup.bash",
            "colcon build --symlink-install --install-base install"
        ]
        for p in post_hooks:
            cmds.append(
                "source /opt/ros/$ROS_DISTRO/setup.bash && "
                "source /ros_ws/install/setup.bash && "
                f"{p}"
            )
        full_cmd = " && ".join(cmds)

        print(f"[build] [{comp_name}] Running build container:")
        print(f"         {full_cmd}")

        # run the unified builder image (created in stage)
        image = comp.image_tag(cfg)
        docker.run(
            image       = image,
            command     = ["bash", "-lc", full_cmd],
            remove      = True,
            tty         = True,
            workdir     = "/ros_ws",
            envs        = {"ROS_DISTRO": cfg.ros_distro},
            volumes     = [(os.path.abspath(ws_root), "/ros_ws", "rw")],
        )

        print(f"[build] '{comp_name}' done; install at {ws_root}/install")

if __name__ == "__main__":
    # usage: build.py [project_root] [--component name]
    args = sys.argv[1:]
    pr   = args[0] if len(args) >= 1 else "."
    comp = args[1] if len(args) == 2 else None
    build_main(pr, comp)
