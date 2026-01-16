# core/docker.py
import sys
import os
import subprocess
import hashlib
import python_on_whales
from python_on_whales import DockerClient
from typing import Optional, List
from colorama import Fore


def is_localhost(host) -> bool:
    """Check if a host is localhost."""
    return host.ip in ('localhost', '127.0.0.1', '::1')


class DockerHelper:
    def __init__(self, host_url=None):
        self.client = DockerClient(host=host_url) if host_url else DockerClient()

    def pull_image_on_host(self, host, image):
        """
        Pulls a Docker image on a host.
        :param host: The remote host object with 'ip', 'port' attributes.
        :param image: The Docker image to pull.
        """
        if is_localhost(host):
            self.client.pull(image)
        else:
            url = f"tcp://{host.ip}:{host.port}"
            cc = DockerClient(host=url)
            cc.pull(image)

    def build_multiarch(self, image_tag, context, dockerfile, platforms, push=True, logger=None):
        """
        Build multi-architecture image.

        Args:
            image_tag: Tag for the built image
            context: Build context directory
            dockerfile: Path to Dockerfile
            platforms: List of target platforms
            push: Whether to push after building
            logger: Optional ComponentLogger for prefixed output
        """
        try:
            if logger:
                # Build docker buildx command manually to capture output
                cmd = [
                    "docker", "buildx", "build",
                    "-f", dockerfile,
                    "--platform", ",".join(platforms),
                    "-t", image_tag,
                    "--progress=plain",
                ]
                if push:
                    cmd.append("--push")
                cmd.append(context)

                # Run with line-by-line output capture
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    universal_newlines=True,
                    bufsize=1
                )

                # Stream output with component prefix
                for line in process.stdout:
                    logger.log_line(line.rstrip())

                process.wait()
                if process.returncode != 0:
                    raise python_on_whales.exceptions.DockerException(
                        command_launched=cmd,
                        return_code=process.returncode
                    )
            else:
                # Use python-on-whales directly (output goes to stdout)
                self.client.buildx.build(
                    context,
                    file=dockerfile,
                    platforms=platforms,
                    tags=[image_tag],
                    push=push,
                    progress='plain',
                    cache=True
                )

        except python_on_whales.exceptions.DockerException as e:
            if logger:
                logger.error(f"Docker build failed: {e}")
            else:
                print(f"Docker error: {e}", file=sys.stderr)
            raise

    def run_builder(self, image, workdir, command, mounts, envs):
        self.client.run(
            image       = image,
            command     = command,
            workdir     = workdir,
            volumes     = mounts,
            envs        = envs,
            remove      = True,
            tty         = True
        )

    def compose_up_local(self, compose_file, services=None, detach=False):
        cc = DockerClient(compose_files=[compose_file])
        if services:
            cc.compose.up(services, detach=detach)
        else:
            cc.compose.up(detach=detach)

    def compose_up_remote(self, host, compose_file, services=None):
        """Start compose services on a host."""
        if is_localhost(host):
            cc = DockerClient(compose_files=[compose_file])
        else:
            url = f"tcp://{host.ip}:{host.port}"
            cc = DockerClient(host=url, compose_files=[compose_file])
        cc.compose.up(detach=False)

    def compose_down_remote(self, host, compose_file):
        """Stop compose services on a host."""
        if is_localhost(host):
            cc = DockerClient(compose_files=[compose_file])
        else:
            url = f"tcp://{host.ip}:{host.port}"
            cc = DockerClient(host=url, compose_files=[compose_file])
        cc.compose.down()

    def build_on_remote_host(self, host, image_tag: str, context: str, dockerfile: str, push: bool = True):
        """
        Build a Docker image on a remote host.

        This is used when build_on_device=true to avoid slow QEMU emulation.
        The build context is synced to the remote host, built there via SSH, and pushed to registry.

        Uses a persistent remote path (~/forge_builds/<project>/<context>/) so rsync
        can perform incremental syncs on subsequent builds.

        Args:
            host: Host object with ip, port, user attributes
            image_tag: Tag for the built image
            context: Local build context directory (e.g., .forge/base or .forge/workspaces/<comp>)
            dockerfile: Path to Dockerfile (relative to context or absolute)
            push: Whether to push after building
        """
        ssh_prefix = f"{host.user}@{host.ip}"

        # Extract project name from context path (context is under .forge/ directory)
        # e.g., /home/user/my-project/.forge/base -> my-project
        context_abs = os.path.abspath(context)
        parts = context_abs.split(os.sep)
        try:
            forge_idx = parts.index('.forge')
            project_name = parts[forge_idx - 1] if forge_idx > 0 else "default"
        except ValueError:
            project_name = "default"

        # Context name (base, or component name)
        context_name = os.path.basename(context)

        # Use persistent path for rsync incremental syncs (not /tmp)
        remote_build_dir = f"forge_builds/{project_name}/{context_name}"

        print(f"[docker] Building on device {host.name} ({host.ip})")
        print(f"[docker] Remote build directory: ~/{remote_build_dir}")

        # 1) Create remote build directory (in user's home, persistent for rsync incremental sync)
        subprocess.run(
            ["ssh", ssh_prefix, f"mkdir -p {remote_build_dir}"],
            check=True
        )

        # 2) Determine dockerfile path relative to context
        if os.path.isabs(dockerfile):
            # If dockerfile is absolute, compute relative path from context
            dockerfile_rel = os.path.relpath(dockerfile, context)
        else:
            dockerfile_rel = dockerfile

        # 3) Sync build context to remote host
        print(f"[docker] Syncing build context to {host.name}:{remote_build_dir}")
        rsync_cmd = [
            "rsync", "-az", "--delete",
            "-e", "ssh",
            f"{context}/",
            f"{ssh_prefix}:{remote_build_dir}/"
        ]
        subprocess.run(rsync_cmd, check=True)

        # 4) Build image on remote host via SSH (not TCP, to ensure same filesystem view)
        print(f"[docker] Building image {image_tag} on {host.name}...")
        build_cmd = f"cd {remote_build_dir} && docker build -f {dockerfile_rel} -t {image_tag} ."

        result = subprocess.run(
            ["ssh", ssh_prefix, build_cmd],
        )
        if result.returncode != 0:
            print(Fore.RED + f"[docker] Build failed on {host.name}")
            raise python_on_whales.exceptions.DockerException(
                command_launched=["ssh", ssh_prefix, build_cmd],
                return_code=result.returncode
            )

        # 5) Push image to registry if requested
        if push:
            print(f"[docker] Pushing image {image_tag} from {host.name}...")
            push_cmd = f"docker push {image_tag}"
            result = subprocess.run(["ssh", ssh_prefix, push_cmd])
            if result.returncode != 0:
                print(Fore.RED + f"[docker] Push failed on {host.name}")
                raise python_on_whales.exceptions.DockerException(
                    command_launched=["ssh", ssh_prefix, push_cmd],
                    return_code=result.returncode
                )

        print(Fore.GREEN + f"[docker] Successfully built {image_tag} on {host.name}")
