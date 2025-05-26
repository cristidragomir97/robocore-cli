# core/sync.py
import os
import subprocess

class SyncManager:
    def __init__(self, cfg):
        self.mount = cfg.mount_root
        self.build = os.path.abspath(cfg.build_dir)

    def rsync_builds(self, host, components):
        for comp in components:
            src = os.path.join(self.build, comp.name, "ros_ws") + "/"
            dst = f"{host.user}@{host.ip}:{self.mount}/{comp.name}/ros_ws/"
            subprocess.run(
                f"ssh {host.user}@{host.ip} 'mkdir -p {self.mount}/{comp.name}/ros_ws'",
                shell=True, check=True
            )
            cmd = (
                f"rsync -az --inplace --no-whole-file --delete "
                f"-e ssh {src} {dst}"
            )
            print(f"[sync] {host.name}: {cmd}")
            subprocess.run(cmd, shell=True, check=True)
