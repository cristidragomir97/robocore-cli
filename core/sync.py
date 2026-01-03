# core/sync.py
import os
import subprocess

class SyncManager:
    def __init__(self, cfg):
        self.build = os.path.join(cfg.root, cfg.build_dir)
        self.workspace_dir = cfg.workspace_dir


    def rsync_builds(self, project_root,  host, components):
        mount_root = host.effective_mount_root
        for comp in components:
            src = os.path.join(self.build, comp.name, self.workspace_dir) + "/"
            if not os.path.isdir(src):
                print(f"[sync] Skipping {comp.name} (no build found at {src})")
                continue
            else:
                print(f"[sync] Syncing {comp.name} build to {host.name} ({host.ip})")
                dst = f"{host.user}@{host.ip}:{mount_root}/{comp.name}/{self.workspace_dir}/"

                # Create directory and ensure proper ownership
                remote_dir = f"{mount_root}/{comp.name}/{self.workspace_dir}"
                setup_cmd = [
                    "ssh", f"{host.user}@{host.ip}",
                    f"mkdir -p {remote_dir} && chown -R {host.user}:{host.user} {mount_root}"
                ]
                print(f"[sync] Setting up remote directory: {remote_dir}")
                subprocess.run(setup_cmd, check=True)

                cmd = [
                    "rsync", "-az", "--inplace", "--no-whole-file", "--delete",
                    "-e", "ssh", src, dst
                ]
                print(f"[sync] {host.name}: {' '.join(cmd)}")
                subprocess.run(cmd, check=True)
