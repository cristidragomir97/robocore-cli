# core/sync.py
import os
import subprocess

def rsync_file(local_path: str, host, remote_path: str):
    cmd = [
        "rsync", "-az", "--inplace", "--no-whole-file",
        local_path,
        f"{host.user}@{host.ip}:{remote_path}"
    ]
    print(f"[deploy] rsync: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)

class SyncManager:
    def __init__(self, cfg):
        self.mount = cfg.mount_root
        self.build = os.path.abspath(cfg.build_dir)


    def rsync_builds(self, project_root,  host, components):
        for comp in components:
            src = os.path.join(project_root, "build", comp.name, "ros_ws") + "/"
            if not os.path.isdir(src):
                print(f"[sync] Skipping {comp.name} (no build found at {src})")
                continue
            else:
                print(f"[sync] Syncing {comp.name} build to {host.name} ({host.ip})")
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
