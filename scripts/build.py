#!/usr/bin/env python3
import os
import sys
import shutil
import socket
import subprocess
import yaml
from python_on_whales import DockerClient
from utils.common import load_config

def ensure_ssh_server():
    """Ensure SSHD is installed & running for SSHFS mounts."""
    if shutil.which('sshd') is None:
        print("[builder] Installing openssh-server...")
        subprocess.run(['sudo','apt-get','update'], check=True)
        subprocess.run(['sudo','apt-get','install','-y','openssh-server'], check=True)
    subprocess.run(['sudo','systemctl','enable','ssh'], check=True)
    subprocess.run(['sudo','systemctl','start','ssh'],  check=True)
    print("[builder] SSH server is running.")

def build_all_components():
    cfg         = load_config()
    client      = DockerClient()
    comp_root   = cfg['components_dir']
    build_root  = os.path.abspath(cfg['build_dir'])
    tag         = cfg.get('tag','latest')

    os.makedirs(build_root, exist_ok=True)
    built = []

    for name in sorted(os.listdir(comp_root)):
        comp_dir = os.path.join(comp_root, name)
        src_dir  = os.path.join(comp_dir, 'ros_ws', 'src')
        meta_f   = os.path.join(comp_dir, 'component.yaml')
        if not (os.path.isdir(src_dir) and os.path.isfile(meta_f)):
            print(f"[builder] Skipping '{name}' (need ros_ws/src/ + component.yaml)")
            continue

        # Load component config
        meta = yaml.safe_load(open(meta_f))

        # Tag for the builder image
        builder_image = f"{cfg['registry']}/{cfg['image_prefix']}_{name}:builder-{tag}"

        # Build workspace under build/<name>/ros_ws
        comp_ws_root  = os.path.join(build_root, name, 'ros_ws')
        src_out       = os.path.join(comp_ws_root, 'src')
        install_out   = os.path.join(comp_ws_root, 'install')

        # Clean previous build entirely
        if os.path.isdir(comp_ws_root):
            print(f"[builder] Cleaning previous build at {comp_ws_root}")
            shutil.rmtree(comp_ws_root)
        os.makedirs(src_out,   exist_ok=True)
        os.makedirs(install_out, exist_ok=True)

        # Copy source into build/ws/src
        print(f"[builder] Copying source for '{name}' → {src_out}")
        shutil.copytree(src_dir, src_out, dirs_exist_ok=True)

        # Run builder container to populate install_out
        print(f"[builder] Building '{name}' in container '{builder_image}'")
        client.run(
            image=builder_image,
            command=[
                "bash", "-lc",
                "source /opt/ros/$ROS_DISTRO/setup.bash && "
                "colcon build --symlink-install --install-base install"
            ],
            remove=True,
            tty=True,
            workdir="/ros_ws",
            envs={"ROS_DISTRO": cfg['ros_distro']},
            volumes=[
                (os.path.abspath(src_out),   "/ros_ws/src",     "ro"),
                (os.path.abspath(install_out), "/ros_ws/install", "rw"),
            ],
        )
        print(f"[builder] '{name}' built; artifacts in {install_out}")
        built.append(name)

    return built

def print_sshfs_instructions(built):
    cfg        = load_config()
    hostname   = socket.gethostname()
    try:
        dev_ip = socket.gethostbyname(hostname)
    except:
        dev_ip = '<DEV_IP>'

    user       = cfg.get('ssh_user', os.getlogin())
    build_root = os.path.abspath(cfg['build_dir'])
    mount_root = cfg['sshfs_mount_root']

    print("\n[builder] On each robot, run:")
    for name in built:
        remote_ws = os.path.join(build_root, name, 'ros_ws')
        local_mnt = os.path.join(mount_root, name)
        print(f"  mkdir -p {local_mnt}")
        print(
            f"  sshfs {user}@{dev_ip}:{remote_ws} {local_mnt} "
            f"-o reconnect,ServerAliveInterval={cfg['sshfs_options']['ServerAliveInterval']} "
            f"-o ServerAliveCountMax={cfg['sshfs_options']['ServerAliveCountMax']}"
        )
    print()

def main():
    # 1) Ensure SSH server to serve build_dir
    try:
        ensure_ssh_server()
    except subprocess.CalledProcessError as e:
        print(f"[error] SSH server setup failed: {e}", file=sys.stderr)
        sys.exit(1)

    # 2) Build all components
    try:
        built = build_all_components()
    except Exception as e:
        print(f"[error] Build step failed: {e}", file=sys.stderr)
        sys.exit(1)

    # 3) Print SSHFS mount commands for deployment

if __name__ == '__main__':
    main()
