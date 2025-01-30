# run_manager.py

import sys
from typing import Dict, List
from rich.console import Console

from entities.brick import Brick
from entities.package import Package
from managers.host import HostManager

console = Console()

class RunManager:
    """
    The RunManager class handles container 'run' logic:
      - Determining if 'ros2 run' or 'ros2 launch' is needed.
      - Creating the final command with parameters.
      - Detecting devices and capabilities from the Brick's config.
      - Calling HostManager to actually run the container.
    """

    def __init__(self, host_manager: HostManager, built_images: Dict[str, str]):
        """
        :param host_manager: A HostManager instance for the target host.
        :param built_images: A dictionary mapping brick_id -> Docker image tag.
        """
        self.host_manager = host_manager
        self.built_images = built_images  # e.g. {"brick_id": "rc-humble-brick_id-amd64"}

    def run_brick_container(self, brick: Brick):
        """
        Given a Brick with a Package, determine whether to run or launch,
        build the final command, detect devices, and run the container on the host.
        """
        pkg = brick.package
        if not pkg:
            console.print(f"[yellow]Brick '{brick.id}' has no package; skipping run.[/yellow]")
            return

        # Decide if we have a 'run' or 'launch' scenario
        if pkg.run:
            # e.g. ros2 run <pkg.run> param1:=val param2:=val
            base_cmd = f"ros2 run {pkg.run}"
        elif pkg.launch:
            # e.g. ros2 launch <pkg.launch> param1:=val param2:=val
            base_cmd = f"ros2 launch {pkg.launch}"
        else:
            # This brick doesn't define a run/launch command => skip
            return

        # Build param expansions from pkg.params
        param_str = self._build_ros_arg_string(pkg.params or [])
        if param_str:
            base_cmd += f" {param_str}"

        # Get the Docker image tag from built_images
        image_tag = self.built_images.get(brick.id)
        if not image_tag:
            console.print(f"[yellow]No built image found for brick '{brick.id}'; skipping run.[/yellow]")
            return

        # Detect devices & capabilities (if device paths appear in params)
        devices, capabilities = self._detect_devices_and_capabilities(pkg.params or [])

        console.print(f"[cyan]Running container for brick '{brick.id}' on host '{self.host_manager.host.name}'[/cyan]")
        self.host_manager.run_container(
            image=image_tag,
            command=base_cmd,
            devices=devices,
            cap_add=capabilities
        )

    def _build_ros_arg_string(self, params: List) -> str:
        """
        Convert a list of param definitions into ROS-style 'param_name:=value' arguments.
        Example:
          params = [{"serial_port": "/dev/ttyUSB0"}, {"baudrate": 115200}]
          => "serial_port:=/dev/ttyUSB0 baudrate:=115200"
        """
        ros_args = []
        for entry in params:
            if isinstance(entry, dict):
                for k, v in entry.items():
                    ros_args.append(f"{k}:={v}")
            # If needed, handle other param forms (strings, booleans, etc.)
        return " ".join(ros_args)

    def _detect_devices_and_capabilities(self, params: List) -> (List[str], List[str]):
        """
        Naive approach: any param that starts with "/dev/" is considered a device.
        We add a default capability 'SYS_ADMIN' (or any others you need).
        """
        devices = []
        capabilities = []

        for entry in params:
            if isinstance(entry, dict):
                for k, v in entry.items():
                    if isinstance(v, str) and v.startswith("/dev/"):
                        devices.append(v)
                        capabilities.append("SYS_ADMIN")  # or whatever capability you need

        # Deduplicate
        return list(set(devices)), list(set(capabilities))
