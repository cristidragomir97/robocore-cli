#!/usr/bin/env python3
import os
import sys
from core.config import Config
from core.docker import DockerHelper
from core.sync   import SyncManager

def deploy_main(project_root: str, simulate: bool=False):
    cfg    = Config.load(project_root)
    sync   = SyncManager(cfg)
    docker = DockerHelper()


    for host in cfg.hosts:
        compose_file = os.path.join(project_root, cfg.compose_file)
        sync.rsync_builds(host, cfg.components)
        print(f"[deploy] Synced builds to {host.name} ({host.ip}), {compose_file} will be used.")
        docker.compose_up_remote(host, compose_file)
    print("[deploy] All services started (streaming logs).")
