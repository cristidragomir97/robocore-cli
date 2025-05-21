#!/usr/bin/env python3
import os
import sys
import getpass
import yaml
import subprocess
from colorama import Fore

CONFIG = "config.yaml"

def sh(cmd: str):
    print(f"$ {cmd}")
    res = subprocess.run(cmd, shell=True)
    if res.returncode:
        sys.exit(f"[init] Command failed ({res.returncode}): {cmd}")

def init_main(project_root: str):
    """
    Interactively create config.yaml and scaffold components/ & build/ directories.
    """
    root = os.path.abspath(project_root)
    os.makedirs(root, exist_ok=True)
    os.chdir(root)

    if os.path.exists(CONFIG):
        print(Fore.RED + f"[init] {CONFIG} already exists, aborting.", file=sys.stderr)
        sys.exit(1)

    print(Fore.CYAN + "Welcome to robocore-cli! Let's set up your project…")

    # 1) ROS settings
    distro = input("ROS distro [humble]: ").strip() or "humble"
    domain = input("ROS domain ID [0]: ").strip() or "0"

    # 2) Docker image settings
    prefix      = input("Image prefix [myrobot]: ").strip() or "myrobot"
    registry    = input("Docker registry [docker.io/username]: ").strip() or "docker.io/username"
    default_base= f"{registry}/{prefix}_base:{distro}"
    base_image  = input(f"Base image [{default_base}]: ").strip() or default_base
    deploy_mode = input("Deploy mode (image/live) [live]: ").strip() or "live"

    # 3) Paths
    build_dir      = input("Build directory [build]: ").strip() or "build"
    components_dir = input("Components directory [components]: ").strip() or "components"
    compose_file   = input("Compose file [docker-compose.yml]: ").strip() or "docker-compose.yml"
    default_mount  = f"/home/{getpass.getuser()}/ros_builds"
    mount_root     = input(f"Mount root on hosts [{default_mount}]: ").strip() or default_mount

    # 4) Hosts
    num_hosts = input("Number of target hosts [1]: ").strip() or "1"
    try:
        n = int(num_hosts)
    except ValueError:
        n = 1

    hosts = []
    for i in range(n):
        print(f"Host #{i+1}:")
        name = input("  name: ").strip()
        ip   = input("  ip/address: ").strip()
        user = input(f"  ssh user [{getpass.getuser()}]: ").strip() or getpass.getuser()
        arch = input("  architecture [amd64]: ").strip() or "amd64"
        hosts.append({"name": name, "ip": ip, "user": user, "arch": arch})

    # 5) Write config.yaml
    cfg = {
        "ros_distro":      distro,
        "ros_domain_id":   int(domain),
        "image_prefix":    prefix,
        "registry":        registry,
        "base_image":      base_image,
        "deploy_mode":     deploy_mode,
        "build_dir":       build_dir,
        "components_dir":  components_dir,
        "compose_file":    compose_file,
        "mount_root":      mount_root,
        "hosts":           hosts,
        "components":      []
    }
    with open(CONFIG, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)

    # 6) Scaffold directories
    os.makedirs(components_dir, exist_ok=True)
    os.makedirs(build_dir, exist_ok=True)

    print(Fore.GREEN + f"[init] Created {CONFIG}, '{components_dir}/', '{build_dir}/'.")
    print(Fore.GREEN + "[init] You can now add components with `robocore-cli component <name> init`")
