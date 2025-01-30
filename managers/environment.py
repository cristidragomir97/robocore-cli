# environment_manager.py

import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from typing import Set

from entities.brick import Brick
from entities.package import Package
from utils import get_last_path_segment

class EnvironmentManager:
    def __init__(self, solution_dir: str):
        self.solution_dir = os.path.abspath(solution_dir)
        self.build_dir = os.path.join(self.solution_dir, "build")
        if not os.path.exists(self.build_dir):
            os.makedirs(self.build_dir)

    # -----------------------
    # Local Directory Creation
    # -----------------------
    def get_build_dir(self) -> str:
        """Returns the build directory."""
        return self.build_dir

    def get_host_dir(self, host_name: str) -> str:
        """Returns subdirectory for a host under build/."""
        h_dir = os.path.join(self.build_dir, host_name)
        if not os.path.exists(h_dir):
            os.makedirs(h_dir)
        return h_dir

    def get_brick_dir(self, host_name: str, brick_id: str) -> str:
        """Returns subdirectory for a brick under a host directory."""
        host_dir = self.get_host_dir(host_name)
        b_dir = os.path.join(host_dir, brick_id)
        if not os.path.exists(b_dir):
            os.makedirs(b_dir)
        return b_dir

    def get_segments_dir(self) -> str:
        """
        Return path to Dockerfile segments directory.
        Adjust as needed to match your project structure.
        """
        current_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(current_dir, "segments")

    # -----------------------
    # Source Retrieval
    # -----------------------
    def prepare_source(self, brick: Brick, target_dir: str, pull: bool = False):
        """
        Prepare source for the given Brick. If it's GitHub, clone or pull.
        If it's local, copy from the solution folder to target_dir.
        """
        pkg = brick.package
        if not pkg:
            print(f"[!] No package defined for brick '{brick.id}', skipping source prep.")
            return

        if pkg.type == "github":
            self._prepare_github_source(pkg, target_dir, pull)
        elif pkg.type == "local":
            self._prepare_local_source(brick, target_dir)
        else:
            raise ValueError(f"Unsupported package type: {pkg.type} for brick {brick.id}")

    def _prepare_github_source(self, package: Package, target_dir: str, pull: bool):
        branch = package.branch or "main"
        repo_url = package.repository

        repo_path = os.path.join(target_dir,get_last_path_segment(repo_url))
        if not repo_url:
            raise ValueError(f"GitHub package requires a repository URL. None found.")

        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        git_dir = os.path.join(repo_path, ".git")
        if os.path.isdir(git_dir):
            # Already a Git repo
            print(f"[*] Found existing Git repo at '{target_dir}'.")
            if pull:
                print(f"[*] Pulling latest changes (branch '{branch}')...")
                subprocess.run(["git", "-C", target_dir, "pull", "origin", branch], check=True)
            else:
                print("[*] --pull not specified, skipping git pull.")
        else:
            # Fresh clone
            print(f"[*] Cloning '{repo_url}' into '{target_dir}' (branch '{branch}')...")
            subprocess.run(["git", "clone", "-b", branch, repo_url, target_dir], check=True)

    def _prepare_local_source(self, brick: Brick, target_dir: str):
        if not brick.package.path:
            raise ValueError("Local package requires a 'path' attribute.")

        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        # Typically: solution_dir/bricks/<brick_id>/ros_ws (adjust if needed)
        origin_dir = os.path.join(self.solution_dir, "bricks", brick.id, "ros_ws")
        if not os.path.isdir(origin_dir):
            raise FileNotFoundError(f"Local origin directory '{origin_dir}' not found.")

        print(f"[*] Copying local package from '{origin_dir}' to '{target_dir}'...")
        shutil.copytree(origin_dir, target_dir, dirs_exist_ok=True)

    # -----------------------
    # Dependency Detection
    # -----------------------
    def detect_dependencies(self, source_dir: str) -> Set[str]:
        """
        Parse package.xml files under 'source_dir' to collect dependencies.
        """
        dependencies = set()
        if not os.path.isdir(source_dir):
            return dependencies  # No source or nonexistent dir => no dependencies

        for root, dirs, files in os.walk(source_dir):
            if "package.xml" in files:
                pkg_xml_path = os.path.join(root, "package.xml")
                try:
                    tree = ET.parse(pkg_xml_path)
                    root_el = tree.getroot()
                    for tag_name in ["depend", "build_depend", "exec_depend", "run_depend", "buildtool_depend"]:
                        for dep_el in root_el.findall(tag_name):
                            dep_name = (dep_el.text or "").strip()
                            if dep_name:
                                dependencies.add(dep_name)
                except ET.ParseError as e:
                    raise ET.ParseError(f"Could not parse '{pkg_xml_path}': {e}") from e

        return dependencies

    # -----------------------
    # Base Image Selection
    # -----------------------
    def choose_base_image(self, dependencies: Set[str], ros_distro: str, registry: str = "") -> str:
        """
        Choose a Docker base image based on discovered dependencies and ROS distro.
        """
        if registry == "local":
            prefix = "localhost:5000/"
        elif registry:
            prefix = "dragomirxyz/"
        else:
            prefix = "dragomirxyz/"

        variant = "base"
        dependency_map = {
            "perception": {"cv_bridge", "vision_opencv", "pcl_ros", "image_transport"},
            "sim": {"gazebo_ros", "gazebo_plugins", "ignition-gazebo"},
            "nav": {"navigation2", "nav2_core", "nav2_bringup", "slam_toolbox"},
        }

        for variant_name, dep_set in dependency_map.items():
            if dependencies.intersection(dep_set):
                variant = variant_name
                break

        return f"{prefix}rcore-{ros_distro}-{variant}"

    # -----------------------
    # Dockerfile Generation
    # -----------------------
    def generate_dockerfile(
        self,
        brick: Brick,
        ros_distro: str,
        source_dir: str,
        dockerfile_path: str,
        base_image: str
    ):
        """
        Generate a Dockerfile by reading segment files, replacing placeholders,
        and injecting RUN commands for preinstall/postinstall.
        """
        segments_dir = self.get_segments_dir()
        base_seg = os.path.join(segments_dir, "base.Dockerfile")
        copy_build_seg = os.path.join(segments_dir, "copy_and_build.Dockerfile")
        entrypoint_seg = os.path.join(segments_dir, "entrypoint.Dockerfile")

        def _read_file(fp: str) -> str:
            with open(fp, 'r') as f:
                return f.read()

        # Read segment contents
        base_content = _read_file(base_seg).replace("{BASE_IMAGE}", base_image)
        copy_build_content = _read_file(copy_build_seg).format(ROS_DISTRO=ros_distro)
        entrypoint_content = _read_file(entrypoint_seg).format(ROS_DISTRO=ros_distro)

        # Gather preinstall, postinstall from the Brick's Package
        pkg = brick.package
        preinstall_cmds = ""
        postinstall_cmds = ""
        if pkg:
            if pkg.preinstall:
                for cmd in pkg.preinstall:
                    preinstall_cmds += f"RUN {cmd}\n"
            if pkg.postinstall:
                for cmd in pkg.postinstall:
                    postinstall_cmds += (
                        f"RUN . /opt/ros/{ros_distro}/setup.sh && "
                        f". /ros2_ws/install/setup.sh && "
                        f"{cmd}\n"
                    )

        dockerfile_content = "\n".join([
            base_content,
            preinstall_cmds,
            copy_build_content,
            postinstall_cmds,
            entrypoint_content
        ])

        with open(dockerfile_path, 'w') as df:
            df.write(dockerfile_content)
        print(f"[*] Dockerfile generated at: {dockerfile_path}")
