#!/usr/bin/env python3
import os
import sys
from typing import Optional, List, Tuple

from core.config   import Config
from core.docker   import DockerHelper
from core.models   import Component

def shell_main(target: str,
               project_root: Optional[str] = None):
    """
    Launch an interactive ROS shell for:
      • A named component
      • The 'common' shared workspace
      • Or a raw filesystem path
    """
    # 1) Determine project root
    pr = os.path.abspath(project_root or ".")
    os.chdir(pr)

    # 2) Load config
    cfg = Config.load(pr)
    docker = DockerHelper()
    image = cfg.base_image

    # 3) Decide what we're opening
    mounts: List[Tuple[str,str,str]] = []
    source_lines = ["source /opt/ros/$ROS_DISTRO/setup.bash"]

    if target == "common":
        # Shell into the common_packages overlay
        common_dir = os.path.join(pr, cfg.common_packages_dir, "ros_ws", "install")
        if not os.path.isdir(common_dir):
            sys.exit(f"[shell] ERROR: no common_packages found at {common_dir}")
        mounts.append((common_dir, "/common_ws/install", "ro"))
        source_lines.append("source /common_ws/install/setup.bash")

    else:
        # Try to find a matching component
        comp: Optional[Component] = None
        for c in cfg.components:
            if c.name == target:
                comp = c
                break

        if comp:
            # 3a) Found component: pick its image
            image = comp.image_tag(cfg)

            # 3b) Common overlay, if any
            if cfg.common_packages:
                common_dir = os.path.join(pr, cfg.common_packages_dir, "ros_ws", "install")
                mounts.append((common_dir, "/common_ws/install", "ro"))
                source_lines.append("source /common_ws/install/setup.bash")

            # 3c) VCS overlay, if any
            if comp.repositories:
                vcs_dir = os.path.join(pr, cfg.build_dir, comp.name, "vcs_ws", "install")
                mounts.append((vcs_dir, "/vcs_ws/install", "ro"))
                source_lines.append("source /vcs_ws/install/setup.bash")

            # 3d) Local workspace overlay, if any
            ros_install = os.path.join(pr, cfg.build_dir, comp.name, "ros_ws", "install")
            if os.path.isdir(ros_install):
                mounts.append((ros_install, "/ros_ws/install", "ro"))
                source_lines.append("source /ros_ws/install/setup.bash")

            # 3e) Always mount the source tree so you can edit it
            src_dir = os.path.join(pr, comp.folder, "ros_ws", "src")
            if os.path.isdir(src_dir):
                mounts.append((src_dir, "/ros_ws/src", "rw"))
        else:
            # 3f) Not a component: treat as path
            host_path = os.path.abspath(target)
            if not os.path.isdir(host_path):
                sys.exit(f"[shell] ERROR: '{host_path}' not found or not a directory")
            mounts.append((host_path, "/ros_ws/src", "rw"))
            source_lines.append("source /ros_ws/src/setup.bash")

    # 4) Assemble the final launch command
    source_script = " && \\\n    ".join(source_lines)
    cmd = f"{source_script} && exec \"$SHELL\""

    print(f"[shell] Launching interactive shell for '{target}' with image '{image}'")
    print(f"[shell] Mounts: {', '.join(f'{m[0]}:{m[1]} ({m[2]})' for m in mounts)}")
    print(f"[shell] Command: {cmd}")

    # 5) Run the container
    #    we rely on DockerHelper to point to the right daemon & buildx builder
    docker.client.run(
        image       = image,
        command     = ["bash", "-lc", cmd],
        interactive = True,
        tty         = True,
        remove      = True,
        workdir     = "/ros_ws",
        envs        = {"ROS_DISTRO": cfg.ros_distro},
        volumes     = mounts
    )

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: rosdock shell <common|component_name|path> [--project-root <root>]", file=sys.stderr)
        sys.exit(1)
    # you could parse a --project-root here if you like
    shell_main(target=sys.argv[1], project_root=".")
