#!/usr/bin/env python3
import os
import sys
import socket
import subprocess
from python_on_whales import DockerClient
from utils.common import load_config, get_hosts

def get_dev_ip():
    """Discover a reachable local IP address for SSH/rsync."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

def copy_builds_to_host(host, cfg, components):
    """
    Rsync build/<component>/ros_ws → host:~/ros_builds/<component>/ros_ws
    using archive, in-place updates, no-whole-file, delete extraneous files, and compression.
    """
    user       = cfg.get('ssh_user', os.getlogin())
    build_root = os.path.abspath(cfg['build_dir'])
    dev_ip     = get_dev_ip()

    for name in components:
        local_ws = os.path.join(build_root, name, 'ros_ws') + '/'
        remote_ws = f"{host.user}@{host.ip}:~/ros_builds/{name}/ros_ws/"
        # ensure remote dir
        subprocess.run(
            f"ssh {host.user}@{host.ip} 'mkdir -p ~/ros_builds/{name}/ros_ws'",
            shell=True,
            check=True
        )
        # rsync
        cmd = (
            "rsync -az --inplace --no-whole-file --delete "
            f"-e ssh {local_ws} {remote_ws}"
        )
        print(f"[deploy] {host.name}: {cmd}")
        subprocess.run(cmd, shell=True, check=True)

def bring_up_containers_on_host(host, cfg):
    """
    Connect to the host's Docker daemon and pull + up the compose stack,
    streaming logs (detach=False).
    """
    base_url = f"tcp://{host.ip}:{cfg.get('docker_port', 2375)}"
    client   = DockerClient(
        host=base_url,
        compose_files=[cfg['compose_file']]
    )


    print(f"[deploy] {host.name}: launching containers (streaming logs)…")
    # default detach=False → streams logs until you Ctrl-C
    client.compose.up()

def deploy_main(project_root: str):
    """
    project_root: path to your solution folder containing config.yaml, build/, etc.
    """
    # Switch to that folder so load_config() sees config.yaml
    os.chdir(project_root)

    cfg        = load_config()
    hosts      = get_hosts(cfg)
    build_root = cfg['build_dir']

    if not os.path.isdir(build_root):
        print(f"[deploy] ERROR: build directory '{build_root}' not found", file=sys.stderr)
        sys.exit(1)

    # components = subdirs under build/ that contain ros_ws/
    components = [
        d for d in sorted(os.listdir(build_root))
        if os.path.isdir(os.path.join(build_root, d, 'ros_ws'))
    ]

    if not hosts:
        print("[deploy] ERROR: No hosts defined in config.yaml", file=sys.stderr)
        sys.exit(1)

    for host in hosts:
        try:
            copy_builds_to_host(host, cfg, components)
            bring_up_containers_on_host(host, cfg)
        except subprocess.CalledProcessError as e:
            print(f"[deploy][error] SSH/rsync failed on {host.name}: {e}", file=sys.stderr)
        except Exception as e:
            print(f"[deploy][error] on {host.name}: {e}", file=sys.stderr)

if __name__ == '__main__':
    # allow: python deploy.py [project_root]
    project_root = sys.argv[1] if len(sys.argv) > 1 else "."
    deploy_main(project_root)
