# host_manager.py

import os
import docker
import paramiko
from typing import Optional, Generator, List

from entities.host import Host

class HostManager:
    def __init__(self, host: Host):
        """
        Manages remote operations on this host (Docker, SSH, etc.).
        """
        self.host = host
        self._docker_client: Optional[docker.DockerClient] = None


    def _connect_docker(self):
        """Initialize a Docker client for the remote host if needed."""
        if self._docker_client is None:
            docker_url = f"tcp://{self.host.ip}:2375"
            self._docker_client = docker.DockerClient(base_url=docker_url)
        return self._docker_client
    
    def list_running_containers(self):
        """
        Lists all running Docker containers on a remote host.

        Args:
            remote_host (str): Docker API address (e.g., "tcp://192.168.1.10:2375").

        Returns:
            list: A list of dictionaries containing container details.
        """
        self._connect_docker()
        try:
            containers = self._docker_client.containers.list(all = True)
            return containers
            
        except Exception as e:
            print(f"Failed to list containers on {self.host.name}: {str(e)}")
            return []

    def build_image(self, build_path: str, dockerfile_name: str, tag: str) -> Generator[str, None, None]:
        """
        Builds a Docker image using the remote daemon on this host.
        :param build_path: Local path containing Dockerfile + context.
        :param dockerfile_name: Name of Dockerfile (e.g., "Dockerfile").
        :param tag: Docker image tag (e.g. "myimage:latest").
        """
        client = self._connect_docker()
        api_client = client.api

        print(f"[*] Building image '{tag}' on '{self.host.name}' from '{build_path}'...")
        build_resp = api_client.build(
            path=build_path,
            dockerfile=dockerfile_name,
            tag=tag,
            decode=True
        )
        for line in build_resp:
            msg = line.get('stream', '').rstrip()
            if msg:
                yield msg

    def run_ssh_command(self, command: str):
        """
        Optional: run a command via SSH on this host.
        """
        print(f"[*] SSH to {self.host.name} => {command}")
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(
                hostname=self.host.ip,
                username=self.host.user,
                password=self.host.password,
                key_filename=self.host.keyfile
            )
            _, stdout, stderr = ssh.exec_command(command)
            for out_line in stdout:
                print(f"[{self.host.name}] {out_line.strip()}")
            for err_line in stderr:
                print(f"[{self.host.name} ERROR] {err_line.strip()}")
            ssh.close()
        except Exception as e:
            print(f"SSH error on {self.host.name}: {e}")

    def run_container(
        self,
        image: str,
        command: str,
        devices: List[str],
        cap_add: List[str]
    ):
        
        """
        Creates and starts a container on the remote Docker host.
        :param image: Docker image tag (e.g. 'myimage:latest').
        :param command: The shell command to run inside the container (e.g., 'ros2 run my_node').
        :param devices: List of host device paths (e.g. ['/dev/ttyUSB0']).
        :param cap_add: List of Linux capabilities to add (e.g. ['SYS_ADMIN']).
        """

        try:
            client = self._connect_docker()

            # Build a container command that runs in a shell so we can parse things like 'ros2 run ... param:=val'
            # If you want direct entrypoint usage, you could pass command as a list, but here's a bash approach:
            shell_cmd = f"/bin/bash -c '{command}'"

            print(f"[green]Starting container on '{self.host.name}' with image='{image}' and command='{shell_cmd}'[/green]")

            # Approach A) Using 'containers.run'
            # The Docker Python SDK doesn't have a top-level 'devices' param in .run,
            # so we might do something like device_requests or the lower-level API.
            # The simplest might be a lower-level approach shown in Approach B below.

            # Approach B) Using the lower-level API calls to pass 'devices' & 'cap_add':
            # 1) Create a host config with capabilities and devices
            device_list = []
            for dev in devices:
                device_list.append({
                    'PathOnHost': dev,
                    'PathInContainer': dev,
                    'CgroupPermissions': 'rwm'
                })

            host_config = client.api.create_host_config(
                cap_add=cap_add,
                devices=device_list,
                # If you want to run in privileged mode, you can do privileged=True
                # or pass additional config as needed.
            )

            # 2) Create the container
            container_def = client.api.create_container(
                image=image,
                command=shell_cmd,
                host_config=host_config,
                tty=True  # or not, depending on your usage
            )

            container_id = container_def.get("Id")
            if not container_id:
                print(f"[red]Failed to create container on host '{self.host.name}'[/red]")
                return

            # 3) Start the container
            client.api.start(container_id)
            short_id = container_id[:12]
            print(f"[*] Container '{short_id}' started on host '{self.host.name}'")
        except Exception as e:
            print(e)