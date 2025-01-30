# host_manager.py

import os
import docker
import paramiko
from typing import Optional

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

    def build_image(self, build_path: str, dockerfile_name: str, tag: str):
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
                print(f"[{self.host.name}] {msg}")

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
