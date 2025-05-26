#!/usr/bin/env python3
import os
import sys
import getpass
import subprocess
import yaml
from colorama import Fore

CONFIG = "config.yaml"

def sh(cmd: str):
    print(f"$ {cmd}")
    res = subprocess.run(cmd, shell=True)
    if res.returncode:
        sys.exit(f"[init] Command failed ({res.returncode}): {cmd}")

def init_main(project_root: str):
    # 1) Prep root
    root = os.path.abspath(project_root)
    os.makedirs(root, exist_ok=True)
    os.chdir(root)

    if os.path.exists(CONFIG):
        print(Fore.RED + f"[init] {CONFIG} already exists, aborting.", file=sys.stderr)
        sys.exit(1)

    print(Fore.CYAN + "Welcome to robocore-cli! Let's set up your project…")

    # 2) Minimal prompts
    distro = input("ROS distro [humble]: ").strip() or "humble"
    domain = input("ROS domain ID [0]: ").strip() or "0"
    registry = input("Docker registry [docker.io/username]: ").strip() or "docker.io/username"
    prefix   = input("Image prefix [myrobot]: ").strip() or "myrobot"

    # 3) Hosts block
    num_hosts = input("Number of target hosts [1]: ").strip() or "1"
    try:
        count = int(num_hosts)
    except ValueError:
        count = 1

    hosts = []
    for i in range(count):
        print(f"Host #{i+1}:")
        name = input("  name: ").strip()
        ip   = input("  ip/address: ").strip()
        user = input(f"  ssh user [{getpass.getuser()}]: ").strip() or getpass.getuser()
        arch = input("  architecture [amd64]: ").strip() or "amd64"
        hosts.append({
            "name": name,
            "ip":   ip,
            "user": user,
            "arch": arch
        })

    # 4) Build the dict and dump to YAML
    cfg = {
        "ros_distro": distro,
        "ros_domain_id": int(domain),
        "registry": registry,
        "image_prefix": prefix,
        "hosts": hosts,
        # placeholders: user will fill these in below
        "common_packages": [],
        "components": []
    }

    with open(CONFIG, "w") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)
        # now append commented placeholders and usage hints
        f.write("\n")
        f.write("# ─── Shared/Common Packages ─────────────────────────────────────\n")
        f.write("# common_packages:\n")
        f.write("#   - name: my_msgs\n")
        f.write("#     folder: common_packages/my_msgs\n")
        f.write("#     repo: https://github.com/you/my_msgs.git\n")
        f.write("#     branch: main\n\n")
        f.write("# ─── Components ────────────────────────────────────────────────\n")
        f.write("# components:\n")
        f.write("#   - name: example_component\n")
        f.write("#     folder: components/example_component\n")
        f.write("#     repositories:\n")
        f.write("#       - url: https://github.com/org/repo.git\n")
        f.write("#         version: main\n")
        f.write("#         folder: repo_folder\n")
        f.write("#     entrypoint: ros2 launch example_component launch.py\n")
        f.write("#     devices: []\n")
        f.write("#     ports: []\n\n")
        f.write("# ─── Next Steps ───────────────────────────────────────────────\n")
        f.write("# 1) mkdir -p components/<your_component>/ros_ws/src\n")
        f.write("# 2) Populate common_packages/ with any shared packages\n")
        f.write("# 3) Edit the placeholders above in config.yaml to list them\n")
        f.write("# 4) Run `robocore-cli stage` to generate Dockerfiles & compose\n")
        f.write("# 5) Run `robocore-cli build` to compile in Docker\n")
        f.write("# 6) Run `robocore-cli deploy` to sync and launch containers\n")

    # 5) Scaffold on-disk dirs
    for d in ("components", "build", "common_packages"):
        os.makedirs(d, exist_ok=True)

    print(Fore.GREEN + "[init] Scaffold complete; see config.yaml for placeholders.")
