#!/usr/bin/env python3
import os
import sys
from python_on_whales import DockerClient
from utils.common import load_config

def shell_main(path: str):
    # Resolve & validate
    host_path = os.path.abspath(path)
    if not os.path.isdir(host_path):
        print(f"[shell] ERROR: '{host_path}' is not a directory", file=sys.stderr)
        sys.exit(1)

    # Load base image + distro from config.yaml
    cfg    = load_config()
    image  = cfg['base_image']
    distro = cfg['ros_distro']

    # Launch interactive container
    docker = DockerClient()
    docker.run(
        image       = image,
        command     = ["bash","-lc",
                       "source /opt/ros/$ROS_DISTRO/setup.bash && bash"],
        interactive = True,
        tty         = True,
        remove      = True,
        workdir     = "/ros_ws",
        envs        = {"ROS_DISTRO": distro},
        volumes     = [(host_path, "/ros_ws", "rw")],
    )

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: shell.py <path-to-ros_ws>", file=sys.stderr)
        sys.exit(1)
    shell_main(sys.argv[1])
