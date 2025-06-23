#!/usr/bin/env python3
import os
import sys
from core.config import Config
from core.docker import DockerHelper
from core.sync   import SyncManager
from core.models import Host

def deploy_main(project_root: str, simulate: bool=False, host_name: str = None):
    cfg    = Config.load(project_root)
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

        sync.rsync_builds(project_root, host, cfg.components)

        print(f"[deploy:{host.name}] Synced builds & compose. Launching containers...")
        docker.compose_up_remote(host, compose_path)
    print("[deploy] All services started (check terminal outputs).")
