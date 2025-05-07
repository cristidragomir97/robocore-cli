#!/usr/bin/env python3
import os
import sys
import socket
import subprocess
from python_on_whales import DockerClient
from utils.common import load_config, get_hosts

def get_dev_ip() -> str:
    """Discover a reachable local IP address for SSH connections."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't actually send packets
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

def remote_run(host, cmd: str, ignore_errors: bool = False):
    """Run a shell command on the remote host via SSH."""
    full = f"ssh {host.user}@{host.ip} \"{cmd}\""
    try:
        subprocess.run(full, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        if ignore_errors:
            print(f"[deploy] Warning (ignored): {e}")
        else:
            raise

def copy_builds_to_host(host, cfg, components):
    """
    Rsync each build/<component>/ros_ws → host:~/ros_builds/<component>/ros_ws
    using archive mode, in-place updates, no-whole-file, delete extraneous files,
    and compression.
    """
    user       = cfg.get('ssh_user', os.getlogin())
    build_root = os.path.abspath(cfg['build_dir'])
    dev_ip     = get_dev_ip()

    for name in components:
        local_ws  = os.path.join(build_root, name, 'ros_ws') + '/'
        remote_ws_dir = f"~/ros_builds/{name}/ros_ws"
        # Ensure the remote directory exists
        remote_run(host, f"mkdir -p {remote_ws_dir}")
        # Rsync command
        cmd = (
            f"rsync -az --inplace --no-whole-file --delete "
            f"-e ssh "
            f"{local_ws} "
            f"{host.user}@{host.ip}:{remote_ws_dir}/"
        )
        print(f"[deploy] {host.name}: {cmd}")
        subprocess.run(cmd, shell=True, check=True)

def bring_up_containers_on_host(host, cfg):
    base_url = f"tcp://{host.ip}:{cfg.get('docker_port', 2375)}"
    client = DockerClient(
        host=base_url,
        compose_files=[cfg['compose_file']]
    )

    #print(f"[deploy] {host.name}: pulling images…")
    #client.compose.pull()                 # pulls services in the compose stack
    print(f"[deploy] {host.name}: docker-compose up -d")
    client.compose.up()        # launches the stack in detached mode
    print(f"[deploy] {host.name}: done\n")


def main():
    cfg        = load_config()
    hosts      = get_hosts(cfg)
    build_root = cfg['build_dir']

    if not os.path.isdir(build_root):
        print(f"[deploy] ERROR: build directory '{build_root}' not found", file=sys.stderr)
        sys.exit(1)

    # Determine components by presence of ros_ws under each build/<name>
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
        except Exception as e:
            print(f"[deploy][error] on {host.name}: {e}", file=sys.stderr)

if __name__ == '__main__':
    main()
