# parser/parser.py

import os
import yaml
from typing import List, Dict, Any, Set, Tuple, Optional

from entities.brick import Brick
from entities.package import Package
from entities.host import Host
from entities.endpoint import Endpoint

def parse_hosts(hosts_config_path: str) -> List[Host]:
    if not os.path.isfile(hosts_config_path):
        raise FileNotFoundError(f"hosts.yaml not found at path: {hosts_config_path}")

    with open(hosts_config_path, 'r') as file:
        data = yaml.safe_load(file)

    overlay_network = data.get('overlay_network', 'shared-network')
    hosts_data = data.get('hosts', [])
    hosts = []

    for hd in hosts_data:
        h = Host(
            name=hd['name'],
            ip=hd['ip'],
            user=hd['user'],
            arch=hd.get('arch', 'amd64'),
            password=hd.get('password'),
            keyfile=hd.get('keyfile'),
            overlay_network=overlay_network
        )
        hosts.append(h)
    return hosts

def parse_config(path: str, hosts: List[Host], base_path: str) -> Tuple[List[Brick], Dict[str, str]]:
    loaded_includes: Set[str] = set()
    bricks: List[Brick] = []
    settings: Dict[str, str] = {}

    def _parse_file(current_path: str, is_main: bool = False):
        if current_path in loaded_includes:
            print(f"[WARNING] Already included '{current_path}', skipping to prevent circular include.")
            return
        if not os.path.isfile(current_path):
            raise FileNotFoundError(f"Configuration file '{current_path}' not found.")

        with open(current_path, 'r') as f:
            data = yaml.safe_load(f)

        if is_main:
            robot_info = data.get('robot', {})
            if not robot_info:
                raise ValueError(f"The main configuration file '{current_path}' lacks a 'robot' section.")

            settings['robot_name'] = robot_info.get('name', 'UnnamedRobot')
            settings['ros_version'] = robot_info.get('ros-version', 'noetic')

            # Copy other fields from 'robot' except bricks, includes
            for k, v in robot_info.items():
                if k not in ('name', 'ros-version', 'bricks'):
                    settings[k] = v

            bricks_data = robot_info.get('bricks', {})
            includes = bricks_data.get('includes', [])
            loaded_includes.add(current_path)

            # Process includes (sub-files that contain additional bricks)
            for inc in includes:
                inc_path = os.path.join(os.path.dirname(current_path), inc)
                _parse_file(inc_path, is_main=False)

            # Process bricks in the main file
            for b_key, b_val in bricks_data.items():
                if b_key == 'includes':
                    continue
                brick = _create_brick(b_key, b_val, hosts, base_path)
                bricks.append(brick)
        else:
            # included file
            bricks_data = data.get('bricks', {})
            if not bricks_data:
                print(f"[WARNING] Included file '{current_path}' has no 'bricks' section.")
                return
            loaded_includes.add(current_path)

            for b_key, b_val in bricks_data.items():
                brick = _create_brick(b_key, b_val, hosts, base_path)
                bricks.append(brick)

    _parse_file(path, is_main=True)
    return bricks, settings

def _create_brick(brick_key: str, brick_value: Dict[str, Any], hosts: List[Host], base_path: str) -> Brick:
    host_name = brick_value.get('host')
    if not host_name:
        raise ValueError(f"Brick '{brick_key}' must have a 'host' defined.")

    # Find the matching host from the list
    host_obj = next((h for h in hosts if h.name == host_name), None)
    if not host_obj:
        raise ValueError(f"Host '{host_name}' for brick '{brick_key}' not found in hosts.yaml.")

    package_data = brick_value.get('package')
    endpoint_data = brick_value.get('endpoints')
    endpoint_obj: Optional[Endpoint] = None

    # Create a single Endpoint if endpoint_data is present
    if endpoint_data and isinstance(endpoint_data, list):
        # If there's a list of endpoints, we only take the FIRST one
        e = endpoint_data[0]
        endpoint_obj = Endpoint(
            type=e.get("type"),
            topic=e.get("topic"),
            message=e.get("message"),
            frame_id=e.get("frame_id")
        )
    elif endpoint_data and isinstance(endpoint_data, dict):
        # If there's a single endpoint object
        endpoint_obj = Endpoint(
            type=endpoint_data.get("type"),
            topic=endpoint_data.get("topic"),
            message=endpoint_data.get("message"),
            frame_id=endpoint_data.get("frame_id")
        )

    # Build the Package if package_data is present
    pkg_obj: Optional[Package] = None
    if package_data:
        source = package_data.get('source', {})
        # Distinguish local vs github
        if 'local' in source:
            pkg_obj = Package(
                build=package_data.get('build', False),
                type='local',
                path=source['local'],
                repository=None,
                branch=None,
                mounts=package_data.get('mounts', []),
                launch=package_data.get('launch'),
                run=package_data.get('run'),
                params=package_data.get('params', []),
                config_files=package_data.get('config_files', []),
                ros_workspace_path=package_data.get('ros_workspace_path'),
                preinstall=package_data.get('preinstall', []),
                postinstall=package_data.get('postinstall', []),
                dockerfile_path=package_data.get('dockerfile_path'),
                docker_tag=package_data.get('docker_tag')
            )
        elif 'github' in source:
            pkg_obj = Package(
                build=package_data.get('build', False),
                type='github',
                path=None,
                repository=source['github'],
                branch=source.get('branch'),
                mounts=package_data.get('mounts', []),
                launch=package_data.get('launch'),
                run=package_data.get('run'),
                params=package_data.get('params', []),
                config_files=package_data.get('config_files', []),
                ros_workspace_path=package_data.get('ros_workspace_path'),
                preinstall=package_data.get('preinstall', []),
                postinstall=package_data.get('postinstall', []),
                dockerfile_path=package_data.get('dockerfile_path'),
                docker_tag=package_data.get('docker_tag')
            )
        else:
            raise ValueError(f"Brick '{brick_key}' package has unsupported 'source': {source}")
    else:
        # No package_data means no package
        print(f"[WARNING] Brick '{brick_key}' has no 'package' entry.")
        pkg_obj = None

    # Finally build the Brick
    brk = Brick(
        id=brick_key,
        host=host_name,
        package=pkg_obj,
        endpoints=endpoint_obj
    )
    return brk

class Parser:
    def __init__(self, base_path: str):
        self.base_path = base_path
        self.robot_config_path = os.path.join(base_path, 'robot.yaml')
        self.hosts_config_path = os.path.join(base_path, 'hosts.yaml')

        # Parse hosts & robot config
        self.hosts = parse_hosts(self.hosts_config_path)
        self.bricks, self.settings = parse_config(self.robot_config_path, self.hosts, self.base_path)

    def bricks_per_host(self, host_name: str):
        return [b for b in self.bricks if b.host == host_name]
