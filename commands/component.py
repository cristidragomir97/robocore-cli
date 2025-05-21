# component.py (excerpt)

import os
import sys
import subprocess
import yaml
from python_on_whales import DockerClient
import tempfile 
import shutil

CONFIG = "config.yaml"

def sh(cmd: str, cwd: str = None):
    print(f"$ {cmd}")
    res = subprocess.run(cmd, shell=True, cwd=cwd)
    if res.returncode:
        sys.exit(f"[component] Command failed ({res.returncode}): {cmd}")

def load_cfg():
    if not os.path.exists(CONFIG):
        sys.exit("[ERROR] config.yaml not found—run `rosdock init` first")
    return yaml.safe_load(open(CONFIG))

def save_cfg(cfg):
    yaml.safe_dump(cfg, open(CONFIG, "w"),
                   default_flow_style=False, sort_keys=False)


def create_package(project_root: str, component: str,
                   repo: str, branch: str = None):
    """
    1) Scaffold into a temp workspace and commit locally
    2) Add as a submodule in the project, commit locally
    3) Register pkg in config.yaml
    (No automatic pushes; user does 'git push' when ready.)
    """
    os.chdir(project_root)
    cfg  = load_cfg()
    comp = next((c for c in cfg["components"] if c["name"]==component), None)
    if not comp:
        sys.exit(f"[component] '{component}' not in config")

    pkg_name    = os.path.splitext(os.path.basename(repo.rstrip("/")))[0]
    comp_folder = comp["folder"]
    ws_src      = os.path.join(comp_folder, "ros_ws", "src")
    dest        = os.path.join(ws_src, pkg_name)

    # ─── ensure parent exists & dest does not ─────────────────────
    os.makedirs(ws_src, exist_ok=True)
    if os.path.exists(dest):
        sys.exit(f"[component] ERROR: target directory already exists: {dest}")

    # ─── 1) scaffold in temp workspace ─────────────────────────────
    tmp = tempfile.mkdtemp(prefix="rosdock_pkg_")
    tmp_ws  = os.path.join(tmp, "ros_ws")
    tmp_src = os.path.join(tmp_ws, "src")
    os.makedirs(tmp_src, exist_ok=True)

    try:
        deps    = input("Dependencies (space-separated, leave blank): ").strip()
        license = input("License [Apache-2.0]: ").strip() or "Apache-2.0"
        desc    = input("Description (optional): ").strip()

        ros2_cmd = (
            f"source /opt/ros/{cfg['ros_distro']}/setup.bash && "
            f"ros2 pkg create --build-type ament_cmake {pkg_name}"
        )
        if deps:    ros2_cmd += f" --dependencies {deps}"
        ros2_cmd += f" --license {license}"
        if desc:    ros2_cmd += f' --description "{desc}"'

        docker = DockerClient()
        docker.run(
            image       = cfg["base_image"],
            command     = ["bash","-lc", ros2_cmd],
            remove      = True,
            tty         = False,
            interactive = False,
            workdir     = "/ros_ws/src",
            envs        = {"ROS_DISTRO": cfg["ros_distro"]},
            volumes     = [(tmp_ws, "/ros_ws", "rw")],
        )

        # Git‐init & local commit in temp
        pkg_path = os.path.join(tmp_src, pkg_name)
        sh("git init", cwd=pkg_path)
        sh("git add .", cwd=pkg_path)
        sh("git commit -m \"first commit\"", cwd=pkg_path)
        target = branch or "main"
        sh(f"git branch -M {target}", cwd=pkg_path)
        sh(f"git remote add origin {repo}", cwd=pkg_path)
        sh(f"git push -u origin {target}", cwd=pkg_path)

        # <- no git push here

    finally:
        shutil.rmtree(tmp)

    # ─── 2) add as submodule in the real project ────────────────────
    sh(f"git submodule add --force {repo} {dest}")
    # stage the submodule entry + .gitmodules
    sh(f"git add .gitmodules {dest}")
    sh(f"git commit -m \"Register '{pkg_name}' as submodule\"")
    # <- no git push here either

    # ─── 3) register in config.yaml ───────────────────────────────
    pkgs = comp.setdefault("packages", [])
    if pkg_name not in pkgs:
        pkgs.append(pkg_name)
        save_cfg(cfg)
        print(f"[component] Registered package '{pkg_name}'")
    else:
        print(f"[component] Package '{pkg_name}' already registered")


def update_submodules(project_root: str, component: str = None):
    """
    Update git submodules to their latest remote commits.
    - If component is None: runs `git submodule update --init --remote --recursive`
    - If component is set: only updates that component’s packages
    """
    os.chdir(project_root)
    cfg = load_cfg()

    if component:
        comp = next((c for c in cfg["components"] if c["name"] == component), None)
        if not comp:
            sys.exit(f"[component] '{component}' not in config")
        pkgs = comp.get("packages", [])
        
        exit
        if not pkgs:
            print(f"[submodules] No packages listed for component '{component}'")
            return
        for pkg in pkgs:
            path = os.path.join(comp["folder"], "ros_ws", "src", pkg)
       
            print(f"[submodules] Updating {path}…")
            # init if needed, then pull latest on configured branch
            subprocess.run(
                ["git", "submodule", "update", "--init", "--remote", path],
                check=True
            )
    else:
        print("[submodules] Updating all submodules…")
        subprocess.run(
            ["git", "submodule", "update", "--init", "--remote", "--recursive"],
            check=True
        )
    print("[submodules] Done.")


def add_package(project_root: str, component: str,
                repo: str, branch: str = None):
    """
    rosdock component <comp> package add --repo <git_url> [--branch <branch>]
    - Derives the package name from the URL
    - Adds it as a submodule (tracking branch if given)
    - Registers in config.yaml
    """
    os.chdir(project_root)
    cfg  = load_cfg()
    comp = next((c for c in cfg.get("components", []) if c["name"] == component), None)
    if not comp:
        sys.exit(f"[component] '{component}' not in config")

    pkg_name = os.path.splitext(os.path.basename(repo.rstrip("/")))[0]
    dest     = os.path.join(comp["folder"], "ros_ws", "src", pkg_name)

    cmd = f"git submodule add"
    if branch:
        cmd += f" -b {branch}"
    cmd += f" {repo} {dest}"
    sh(cmd)
    print(f"[component] Added submodule {repo}")

    pkgs = comp.setdefault("packages", [])
    if pkg_name not in pkgs:
        pkgs.append(pkg_name)
        save_cfg(cfg)
        print(f"[component] Registered package '{pkg_name}'")
    else:
        print(f"[component] Package '{pkg_name}' already registered")