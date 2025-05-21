#!/usr/bin/env python3
import os
import sys
import socket
import subprocess
from python_on_whales import DockerClient
from utils.common import (
    load_config,
    get_hosts,
    get_components
)

def get_dev_ip():
    """Discover a reachable local IP address for SSH/rsync."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

def copy_builds_to_host(host, cfg, component_names):
    """
    Rsync each build/<component>/ros_ws → host:{mount_root}/{component}/ros_ws
    """
    mount = cfg['mount_root']
    build_root = os.path.abspath(cfg['build_dir'])
    for name in component_names:
        local_ws  = os.path.join(build_root, name, 'ros_ws') + '/'
        remote_ws = f"{host.user}@{host.ip}:{mount}/{name}/ros_ws/"
        # ensure the remote folder exists
        subprocess.run(
            f"ssh {host.user}@{host.ip} 'mkdir -p {mount}/{name}/ros_ws'",
            shell=True, check=True
        )
        cmd = (
            f"rsync -az --inplace --no-whole-file --delete "
            f"-e ssh {local_ws} {remote_ws}"
        )
        print(f"[deploy] {host.name}: {cmd}")
        subprocess.run(cmd, shell=True, check=True)

def bring_up_containers_on_host(host, cfg):
    """
    Pull & up the compose stack on the remote Docker daemon,
    streaming logs until you Ctrl-C.
    """
    client = DockerClient(
        host=f"tcp://{host.ip}:{cfg.get('docker_port',2375)}",
        compose_files=[cfg['compose_file']]
    )
    print(f"[deploy] {host.name}: pulling images…")
    client.compose.pull()
    print(f"[deploy] {host.name}: starting services (streaming logs)…")
    client.compose.up()  # default detach=False → live logs

def bring_down_containers_on_host(host, cfg):
    """
    Tear down (stop & remove) the compose stack cleanly.
    """
    client = DockerClient(
        host=f"tcp://{host.ip}:{cfg.get('docker_port',2375)}",
        compose_files=[cfg['compose_file']]
    )
    print(f"[deploy] {host.name}: tearing down stack…")
    client.compose.down()

def deploy_main(project_root: str):
    os.chdir(project_root)
    cfg   = load_config()
    hosts = get_hosts(cfg)
    comps = [c.name for c in get_components(cfg)]

    # sanity checks
    if not hosts:
        print("[deploy] ERROR: no hosts defined in config.yaml", file=sys.stderr)
        sys.exit(1)
    if not comps:
        print("[deploy] ERROR: no components defined in config.yaml", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(cfg['build_dir']):
        print(f"[deploy] ERROR: build directory '{cfg['build_dir']}' missing", file=sys.stderr)
        sys.exit(1)

    try:
        for host in hosts:
            copy_builds_to_host(host, cfg, comps)
            bring_up_containers_on_host(host, cfg)

        print("[deploy] All services exited cleanly.")
    except KeyboardInterrupt:
        print("\n[deploy] Ctrl-C caught, tearing down all stacks…")
        for host in hosts:
            try:
                bring_down_containers_on_host(host, cfg)
            except Exception as e:
                print(f"[deploy][error] failed to tear down {host.name}: {e}", file=sys.stderr)
        sys.exit(0)
    except subprocess.CalledProcessError as e:
        print(f"[deploy][error] SSH/rsync failed: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"[deploy][error] Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    project_root = sys.argv[1] if len(sys.argv) > 1 else "."
    deploy_main(project_root)
