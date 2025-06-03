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
        common_dir = os.path.join(pr, cfg.common_packages_dir)
        print(f"[shell] Using common_packages from {common_dir}")
        if not os.path.isdir(common_dir):
            sys.exit(f"[shell] ERROR: no common_packages found at {common_dir}")
        else:
            if os.path.isempty(common_dir):
                print(f"[shell] ERROR: common_packages directory '{common_dir}' is empty")
            else:
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
                source_lines.append("source /common_ws/install/setup.bash")

            # 3c) VCS overlay, if any
            if comp.repositories:
                source_lines.append("source /vcs_ws/install/setup.bash")

            ros_ws_src = os.path.join(pr, "components", comp.name, "ros_ws", "src")
            os.makedirs(ros_ws_src, exist_ok=True)  # ← create it if needed
            mounts.append((ros_ws_src, "/ros_ws/src", "rw"))

            ros_ws_install = os.path.join(pr, "components", comp.name, "ros_ws", "install")
            if os.path.isdir(ros_ws_install):
                source_lines.append("source /ros_ws/install/setup.bash")
        else:
            # 3f) Not a component: treat as path
            host_path = os.path.abspath(target)
            if not os.path.isdir(host_path):
                mount.append((host_path, "/ros_ws/src", "rw"))
                sys.exit(f"[shell] ERROR: '{host_path}' not found or not a directory")
            mounts.append((host_path, "/ros_ws/src", "rw"))

            # might be empty, which is fine when something was just previsioned
            #source_lines.append("source /ros_ws/src/setup.bash")

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
        hostname = f"rosdock_shell_{target}",
        envs        = {"ROS_DISTRO": cfg.ros_distro},
        volumes     = mounts
    )

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: rosdock shell <common|component_name|path> [--project-root <root>]", file=sys.stderr)
        sys.exit(1)
    # you could parse a --project-root here if you like
    shell_main(target=sys.argv[1], project_root=".")
