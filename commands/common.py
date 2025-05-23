import os
import sys
import shutil
import tempfile
import subprocess
import yaml
from python_on_whales import DockerClient
from utils.common import sh, load_cfg, save_cfg


def create_common_package(project_root: str, repo: str, branch: str = None):
    """
    rosdock common package create --repo <git_url> [--branch <branch>]
    
    1) Scaffold a new ROS2 package in a temp workspace
    2) Init git, commit, branch, add remote, push initial commit
    3) Clean up temp
    4) git submodule add --force into common_packages/<pkg>
    5) Register in config.yaml under common_packages
    """
    # 1) cd into project root
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    cfg = load_cfg()
    common_list = cfg.setdefault("common_packages", [])

    # derive pkg_name and final destination
    pkg_name = os.path.splitext(os.path.basename(repo.rstrip("/")))[0]
    dest     = os.path.join("common_packages", pkg_name)
    parent   = os.path.dirname(dest)
    os.makedirs(parent, exist_ok=True)

    # ensure we don’t clobber an existing folder
    if os.path.exists(dest):
        sys.exit(f"[common] ERROR: destination already exists: {dest}")

    # 2) scaffold in a temporary workspace
    tmpdir   = tempfile.mkdtemp(prefix="rosdock_common_")
    tmp_ws   = os.path.join(tmpdir, "ros_ws")
    tmp_src  = os.path.join(tmp_ws, "src")
    os.makedirs(tmp_src, exist_ok=True)

    try:
        # prompt metadata
        deps    = input("Dependencies (space-separated, leave blank): ").strip()
        license = input("License [Apache-2.0]: ").strip() or "Apache-2.0"
        desc    = input("Description (optional): ").strip()

        # ros2 pkg create
        distro  = cfg["ros_distro"]
        ros2_cmd = (
            f"source /opt/ros/{distro}/setup.bash && "
            f"ros2 pkg create --build-type ament_cmake {pkg_name}"
        )
        if deps:
            ros2_cmd += f" --dependencies {deps}"
        ros2_cmd += f" --license {license}"
        if desc:
            ros2_cmd += f" --description \"{desc}\""

        print(f"[common] Scaffolding package in temp dir: {tmp_src}")
        docker = DockerClient()
        docker.run(
            image       = cfg["base_image"],
            command     = ["bash", "-lc", ros2_cmd],
            remove      = True,
            tty         = False,
            interactive = False,
            workdir     = "/ros_ws/src",
            envs        = {"ROS_DISTRO": distro},
            volumes     = [(tmp_ws, "/ros_ws", "rw")],
        )

        # 3) git init, commit, branch, remote, push
        pkg_path = os.path.join(tmp_src, pkg_name)
        sh("git init", cwd=pkg_path)
        sh("git add .", cwd=pkg_path)
        sh("git commit -m \"first commit\"", cwd=pkg_path)
        target = branch or "main"
        sh(f"git branch -M {target}", cwd=pkg_path)
        sh(f"git remote add origin {repo}", cwd=pkg_path)
        sh(f"git push -u origin {target}", cwd=pkg_path)

    finally:
        # 4) clean up temp workspace
        shutil.rmtree(tmpdir)

    # 5) add as submodule into common_packages/
    sh(f"git submodule add --force {repo} {dest}")

    # 6) register in config.yaml
    common_list.append({
        "name":   pkg_name,
        "folder": dest,
        "repo":   repo,
        "branch": branch
    })
    save_cfg(cfg)

    print(f"[common] Registered common package '{pkg_name}'")


def add_common(project_root: str, repo: str, branch: str = None):
    """
    rosdock common add --repo <git_url> [--branch <branch>]
    """
    # 1) Switch to project root
    os.chdir(project_root)

    # 2) Load & update config
    cfg  = load_cfg()
    common = cfg.setdefault("common_packages", [])

    # 3) Derive name & dest
    name = os.path.splitext(os.path.basename(repo.rstrip("/")))[0]
    dest = os.path.join("common_packages", name)

    # ensure the parent folder exists
    os.makedirs(os.path.dirname(dest), exist_ok=True)

    # 4) Add as submodule
    cmd = f"git submodule add {'-b '+branch if branch else ''} {repo} {dest}"
    sh(cmd)

    # 5) Register in config and save
    common.append({
        "name":   name,
        "folder": dest,
        "repo":   repo,
        "branch": branch
    })
    save_cfg(cfg)

    print(f"[common] Registered shared package '{name}'")
