# core/docker.py
import sys
import subprocess
import python_on_whales
from python_on_whales import DockerClient
from typing import Optional


def is_localhost(host) -> bool:
    """Check if a host is localhost."""
    return host.ip in ('localhost', '127.0.0.1', '::1')


class DockerHelper:
    def __init__(self, host_url=None):
        self.client = DockerClient(host=host_url) if host_url else DockerClient()

    def pull_image_on_host(self, host, image):
        """
        Pulls a Docker image on a remote host.
        :param host: The remote host object with 'ip' and 'port' attributes.
        :param image: The Docker image to pull.
        """
        # Check if host is localhost
        is_localhost = host.ip in ('localhost', '127.0.0.1', '::1')

        if is_localhost:
            # Use local Docker daemon
            cc = DockerClient()
        else:
            # Connect to remote Docker daemon via TCP
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
        # Check if host is localhost
        is_localhost = host.ip in ('localhost', '127.0.0.1', '::1')

        if is_localhost:
            # Use local Docker daemon
            cc = DockerClient(compose_files=[compose_file])
        else:
            # Connect to remote Docker daemon via TCP
            url = f"tcp://{host.ip}:{host.port}"
            cc = DockerClient(host=url, compose_files=[compose_file])

        cc.compose.up(detach=False)

    def compose_down_remote(self, host, compose_file):
        # Check if host is localhost
        is_localhost = host.ip in ('localhost', '127.0.0.1', '::1')

        if is_localhost:
            # Use local Docker daemon
            cc = DockerClient(compose_files=[compose_file])
        else:
            # Connect to remote Docker daemon via TCP
            url = f"tcp://{host.ip}:{host.port}"
            cc = DockerClient(host=url, compose_files=[compose_file])

        cc.compose.down()
