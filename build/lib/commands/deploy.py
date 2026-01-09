#!/usr/bin/env python3
import os
import sys
import signal
from colorama import Fore
from core.config import Config
from core.docker import DockerHelper
from core.sync   import SyncManager
from core.models import Host
from python_on_whales.exceptions import DockerException

def deploy_main(project_root: str, simulate: bool=False, host_name: str = None, config_file: str = 'config.yaml'):
    cfg    = Config.load(project_root, config_file=config_file)
    sync   = SyncManager(cfg)
    docker = DockerHelper()

    hosts = cfg.hosts
    if host_name:
        host = next((h for h in hosts if h.name == host_name), None)
        if not host:
            sys.exit(f"[deploy] ERROR: Host '{host_name}' not found in config.")
        hosts = [host]

    for host in hosts:
        compose_filename = f"docker-compose.{host.name}.yaml"
        compose_path = os.path.join(project_root, compose_filename)
        print(compose_path)
        if not os.path.exists(compose_path):
            print(f"[deploy:{host.name}] WARNING: Compose file '{compose_filename}' not found, skipping.")
            continue

        # Filter components for this host
        host_components = [c for c in cfg.components if c.runs_on == host.name]

        # Only sync builds for components that run on this host
        sync.rsync_builds(project_root, host, host_components)

        # Pull component images on host
        for comp in host_components:
            image = comp.image_tag(cfg)
            print(f"[deploy:{host.name}] Pulling {image}...")
            try:
                docker.pull_image_on_host(host, image)
            except DockerException:
                print(Fore.RED + f"[deploy:{host.name}] Failed to pull image on host '{host.name}' ({host.ip})", file=sys.stderr)
                raise

        print(f"[deploy:{host.name}] Synced builds & compose. Launching containers...")
        try:
            docker.compose_up_remote(host, compose_path)
        except KeyboardInterrupt:
            print(f"\n[deploy:{host.name}] Interrupted by user. Containers may still be running.")
            print(f"[deploy:{host.name}] To stop: docker compose -f {compose_filename} down")
            sys.exit(130)  # Standard exit code for SIGINT
    print("[deploy] All services started (check terminal outputs).")
