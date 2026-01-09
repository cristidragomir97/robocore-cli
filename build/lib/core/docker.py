# core/docker.py
import python_on_whales
from python_on_whales import DockerClient

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

    def build_multiarch(self, image_tag, context, dockerfile, platforms, push=True):
        try:
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
            print(f"Docker error: {e}")
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
        cc.compose.up(service_names=services or [], detach=detach)

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
