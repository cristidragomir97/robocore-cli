#!/usr/bin/env python3
import os, sys, socket, subprocess
from python_on_whales import DockerClient
from utils.common import load_config, get_hosts

def get_dev_ip():
    # pick a reachable local IP
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()
    return ip

def remote_run(host, cmd, ignore_errors=False):
    full = f"ssh {host.user}@{host.ip} \"{cmd}\""
    try:
        subprocess.run(full, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        if not ignore_errors:
            raise
        print(f"[deploy] Warning (ignored): {e}")

def unmount_previous(host, components):
    """Tear down any old ~/ros_builds/<comp> mounts."""
    for name in components:
        mnt = f"$HOME/ros_builds/{name}"
        remote_run(host, f"fusermount -u {mnt}", ignore_errors=True)
        remote_run(host, f"umount {mnt}",     ignore_errors=True)

def mount_builds_on_host(host, cfg, components):
    """Create ~/ros_builds/<comp> and mount build/<comp>/ros_ws there."""
    user      = cfg.get('ssh_user', os.getlogin())
    dev_ip    = get_dev_ip()
    build_dir = os.path.abspath(cfg['build_dir'])

    # Clean out any stale mounts
    unmount_previous(host, components)

    for name in components:
        remote_mnt = f"$HOME/ros_builds/{name}"
        # mkdir -p ~/ros_builds/<name>
        remote_run(host, f"mkdir -p {remote_mnt}")
        # sshfs mount
        local_ws = os.path.join(build_dir, name, 'ros_ws')
        cmd = (
            f"sshfs {user}@{dev_ip}:{local_ws} {remote_mnt} "
            f"-o reconnect "
            f"-o ServerAliveInterval={cfg['sshfs_options']['ServerAliveInterval']} "
            f"-o ServerAliveCountMax={cfg['sshfs_options']['ServerAliveCountMax']}"
        )
        print(f"[deploy] {host.name}: {cmd}")
        remote_run(host, cmd)

def bring_up_containers_on_host(host, cfg):
    """Pull images and docker-compose up via remote Docker API."""
    base_url = f"tcp://{host.ip}:{cfg.get('docker_port', 2375)}"
    client   = DockerClient(base_url=base_url)
    compose  = cfg['compose_file']

    print(f"[deploy] {host.name}: pulling images…")
    client.compose.pull(file=compose)
    print(f"[deploy] {host.name}: docker-compose up -d")
    client.compose.up(detach=True, file=compose)
    print(f"[deploy] {host.name}: done\n")

def main():
    cfg        = load_config()
    hosts      = get_hosts(cfg)
    build_root = cfg['build_dir']

    if not os.path.isdir(build_root):
        print(f"[deploy] build directory '{build_root}' not found", file=sys.stderr)
        sys.exit(1)

    # components = any folder under build/ that contains ros_ws/
    components = [
        d for d in sorted(os.listdir(build_root))
        if os.path.isdir(os.path.join(build_root, d, 'ros_ws'))
    ]

    if not hosts:
        print("[deploy] No hosts defined in config.yaml", file=sys.stderr)
        sys.exit(1)

    for host in hosts:
        try:
            mount_builds_on_host(host, cfg, components)
            bring_up_containers_on_host(host, cfg)
        except Exception as e:
            print(f"[deploy][error] on {host.name}: {e}", file=sys.stderr)

if __name__ == '__main__':
    main()
