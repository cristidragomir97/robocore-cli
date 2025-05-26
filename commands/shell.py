#!/usr/bin/env python3
import os
import sys
from core.config import Config
from core.docker import DockerHelper

def shell_main(path: str):
    host_path = os.path.abspath(path)
    if not os.path.isdir(host_path):
        print(f"[shell] ERROR: '{host_path}' is not a directory", file=sys.stderr)
        sys.exit(1)

    cfg   = Config.load(".")
    docker = DockerHelper()
    docker.client.run(
        image       = cfg.base_image,
        command     = ["bash","-lc",
                       "source /opt/ros/$ROS_DISTRO/setup.bash && bash"],
        interactive = True,
        tty         = True,
        remove      = True,
        workdir     = "/ros_ws",
        envs        = {"ROS_DISTRO": cfg.ros_distro},
        volumes     = [(host_path, "/ros_ws", "rw")]
    )
