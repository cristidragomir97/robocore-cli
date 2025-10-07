#!/usr/bin/env python3
import os
import sys
import shutil
import subprocess
from core.config import Config

def clean_main(project_root: str, remote: bool = False, local: bool = True):
    """
    Clean build artifacts and managed workspaces.

    Args:
        project_root: Path to the project root
        remote: Clean remote hosts as well
        local: Clean local artifacts (default True)
    """
    project_root = os.path.abspath(project_root)
    cfg = Config.load(project_root)

    # Clean local .robocore directory
    if local:
        robocore_dir = os.path.join(project_root, ".robocore")
        if os.path.exists(robocore_dir):
            print(f"[clean] Removing local {robocore_dir}")
            shutil.rmtree(robocore_dir)
            print(f"[clean] ✓ Removed local .robocore")
        else:
            print(f"[clean] Local .robocore directory not found, skipping")

    if remote:
        # Clean remote hosts
        for host in cfg.hosts:
            print(f"[clean:{host.name}] Cleaning remote directories...")

            # Stop and remove containers first
            try:
                compose_file = os.path.join(project_root, f"docker-compose.{host.name}.yaml")
                if os.path.exists(compose_file):
                    print(f"[clean:{host.name}] Stopping containers...")
                    subprocess.run(
                        f"ssh {host.user}@{host.ip} 'cd {cfg.mount_root} && docker compose -f docker-compose.{host.name}.yaml down 2>/dev/null || true'",
                        shell=True
                    )
            except Exception as e:
                print(f"[clean:{host.name}] Warning: Failed to stop containers: {e}")

            # Determine remote project path (assuming same relative structure)
            remote_robocore = f"~/{os.path.basename(project_root)}/.robocore"

            # Remove both mount_root and .robocore directories
            try:
                cmd = f"ssh {host.user}@{host.ip} 'rm -rf {cfg.mount_root} {remote_robocore}'"
                print(f"[clean:{host.name}] Running: {cmd}")
                subprocess.run(cmd, shell=True, check=True)
                print(f"[clean:{host.name}] ✓ Removed {cfg.mount_root} and {remote_robocore}")
            except subprocess.CalledProcessError as e:
                print(f"[clean:{host.name}] ERROR: Failed to clean remote: {e}")

    print("[clean] Done")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Clean build artifacts and workspaces")
    parser.add_argument("-p", "--project-root", default=".", help="Path to project root")
    parser.add_argument("-r", "--remote", action="store_true", help="Clean remote hosts")
    parser.add_argument("--local-only", action="store_true", help="Clean only local artifacts")

    args = parser.parse_args()

    local = not args.remote or not args.local_only
    remote = args.remote

    clean_main(args.project_root, remote=remote, local=local)
