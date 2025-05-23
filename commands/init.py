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
    Interactively create config.yaml and scaffold:
      - components/
      - build/
      - common_packages/
    with our new overlay-based workflow.
    """
    # Prepare project root
    root = os.path.abspath(project_root)
    os.makedirs(root, exist_ok=True)
    os.chdir(root)

    # Bail if already initialized
    if os.path.exists(CONFIG):
        print(Fore.RED + f"[init] {CONFIG} already exists, aborting.", file=sys.stderr)
        sys.exit(1)

    print(Fore.CYAN + "Welcome to robocore-cli! Let's set up your project…")

    # 1) ROS settings
    distro = input("ROS distro [humble]: ").strip() or "humble"
    domain = input("ROS domain ID [0]: ").strip() or "0"

    # 2) Docker registry & naming
    prefix      = input("Image prefix [myrobot]: ").strip() or "myrobot"
    registry    = input("Docker registry [docker.io/username]: ").strip() or "docker.io/username"
    # dynamic base image: registry/prefix_base:distro
    default_base = f"{registry}/{prefix}_base:{distro}"
    base_image   = input(f"Base (ROS2) image [{default_base}]: ").strip() or default_base
    deploy_mode  = input("Deploy mode (image/live) [live]: ").strip() or "live"

    # 3) Directories & files
    build_dir       = input("Build directory [build]: ").strip() or "build"
    components_dir  = input("Components directory [components]: ").strip() or "components"
    common_pkg_dir  = input("Common-packages directory [common_packages]: ").strip() or "common_packages"
    compose_file    = input("Compose file [docker-compose.yml]: ").strip() or "docker-compose.yml"

    # 4) Extra apt packages (comma-separated)
    extras = input("Extra apt packages (comma-separated, optional): ").strip()
    extra_apt = [p.strip() for p in extras.split(",") if p.strip()]

    # 5) Hosts
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

    # 6) Write config.yaml
    cfg = {
        "ros_distro"      : distro,
        "ros_domain_id"   : int(domain),
        "image_prefix"    : prefix,
        "registry"        : registry,
        "base_image"      : base_image,
        "deploy_mode"     : deploy_mode,
        "build_dir"       : build_dir,
        "components_dir"  : components_dir,
        "common_packages" : [],           # start empty
        "extra_apt"       : extra_apt,
        "compose_file"    : compose_file,
        "mount_root"      : f"/home/{getpass.getuser()}/ros_builds",
        "docker_port"     : 2375,
        "hosts"           : hosts,
        "components"      : []            # no components yet
    }
    with open(CONFIG, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)

    # 7) Scaffold directories
    for d in (components_dir, build_dir, common_pkg_dir):
        os.makedirs(d, exist_ok=True)

    print(Fore.GREEN + f"[init] Created:")
    print(f"  • {CONFIG}")
    print(f"  • {components_dir}/")
    print(f"  • {build_dir}/")
    print(f"  • {common_pkg_dir}/")
    print(Fore.GREEN + "[init] Now add your first component:")
    print("    robocore-cli component init <name>")

if __name__=='__main__':
    if len(sys.argv)!=2:
        print("Usage: init.py <project-root>", file=sys.stderr)
        sys.exit(1)
    init_main(sys.argv[1])
