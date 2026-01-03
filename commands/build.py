#!/usr/bin/env python3
import os
import sys
import shutil
import subprocess
from typing import Optional, List, Tuple

from python_on_whales import DockerClient
from colorama import Fore
from core.config   import Config
from core.docker   import is_localhost
from core.models   import Component, Host

def resolve_source_packages(comp: Component, cfg: Config) -> List[Tuple[str, str]]:
    """
    Resolve source paths into a list of (absolute_path, package_name) tuples.
    """
    packages = []
    source_paths = comp.get_source_paths()

    for source_path in source_paths:
        abs_source = os.path.join(cfg.root, source_path)
        if not os.path.exists(abs_source):
            continue

        if not os.path.isdir(abs_source):
            continue

        # Check if this is a ROS package
        if os.path.exists(os.path.join(abs_source, "package.xml")):
            package_name = os.path.basename(abs_source)
            packages.append((abs_source, package_name))
        # Check if this is a workspace src directory containing packages
        elif os.path.exists(os.path.join(abs_source, "src")):
            src_contents = os.path.join(abs_source, "src")
            for pkg in os.listdir(src_contents):
                pkg_path = os.path.join(src_contents, pkg)
                if os.path.isdir(pkg_path) and os.path.exists(os.path.join(pkg_path, "package.xml")):
                    packages.append((pkg_path, pkg))

    return packages

def build_on_remote(comp: Component, cfg: Config, host: Host, source_packages: List[Tuple[str, str]]):
    """
    Build a component on a remote host.
    1) Rsync source packages to remote host
    2) Run docker build command via SSH
    3) Leave build artifacts on remote (used by launch)
    """
    comp_name = comp.name
    remote_build_root = f"{host.effective_mount_root}/{comp_name}/{cfg.workspace_dir}"
    remote_src = f"{remote_build_root}/src"

    print(f"[build:{comp_name}] Building on device {host.name} ({host.ip})")

    # Create remote directory structure
    ssh_prefix = f"{host.user}@{host.ip}"
    subprocess.run(
        ["ssh", ssh_prefix, f"mkdir -p {remote_src}"],
        check=True
    )

    # Rsync each source package to remote
    print(f"[build:{comp_name}] Syncing {len(source_packages)} packages to {host.name}...")
    for abs_source, pkg_name in source_packages:
        remote_dest = f"{ssh_prefix}:{remote_src}/{pkg_name}/"
        subprocess.run(
            ["rsync", "-az", "--delete", "-e", "ssh", f"{abs_source}/", remote_dest],
            check=True
        )
        print(f"  - synced {pkg_name}")

    # Build command
    post_hooks = comp.postinstall or []
    cmds = [
        "source /opt/ros/$ROS_DISTRO/setup.bash",
        "colcon build --symlink-install --install-base install"
    ]
    for p in post_hooks:
        cmds.append(
            "source /opt/ros/$ROS_DISTRO/setup.bash && "
            "source /ros_ws/install/setup.bash && "
            f"{p}"
        )
    full_cmd = " && ".join(cmds)

    # Run build via remote Docker
    image = comp.image_tag(cfg)
    docker_url = f"tcp://{host.ip}:{host.port}"

    print(f"[build:{comp_name}] Running build on {host.name}...")
    docker_cmd = [
        "docker", "-H", docker_url, "run", "--rm",
        "-v", f"{remote_build_root}:/ros_ws:rw",
        "-w", "/ros_ws",
        "-e", f"ROS_DISTRO={cfg.ros_distro}",
        image,
        "bash", "-lc", full_cmd
    ]

    result = subprocess.run(docker_cmd)
    if result.returncode != 0:
        print(Fore.RED + f"[build:{comp_name}] Build failed on {host.name}")
        sys.exit(1)

    print(Fore.GREEN + f"[build:{comp_name}] Done (artifacts at {host.name}:{remote_build_root}/install)")


def build_main(project_root: str, component: Optional[str] = None, config_file: str = 'config.yaml'):
    """
    For each component that has local source packages:
      1) Copy sources → build/<comp>/ros_ws/src
      2) Run any postinstall hooks
      3) Invoke the builder image (already staged) to colcon build → install

    If the target host has build_on_device=true, the build runs on the remote device.
    """
    # 1) chdir into project
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    # 2) load config & pick components
    cfg   = Config.load(project_root, config_file=config_file)
    docker = DockerClient()
    hosts_map = {h.name: h for h in cfg.hosts}
    comps = cfg.filter_components(name=component)
    if not comps:
        print(f"[build] No components to build (filter={component})")
        return

    build_root = os.path.abspath(cfg.build_dir)
    os.makedirs(build_root, exist_ok=True)

    # 3) loop
    for comp in comps:
        # Check if this component's host wants on-device builds
        host = hosts_map.get(comp.runs_on)
        if host and host.build_on_device and not is_localhost(host):
            # Resolve source packages
            source_packages = resolve_source_packages(comp, cfg)
            if not source_packages:
                print(f"[build] Skipping '{comp.name}' (no source packages found)")
                continue
            build_on_remote(comp, cfg, host, source_packages)
            continue
        comp_name = comp.name

        # Resolve source packages
        source_packages = resolve_source_packages(comp, cfg)

        # only build if there's local source
        if not source_packages:
            print(f"[build] Skipping '{comp_name}' (no source packages found)")
            continue

        # prepare a clean workspace
        ws_root = os.path.join(build_root, comp_name, cfg.workspace_dir)
        ws_src = os.path.join(ws_root, 'src')

        # copy in your sources
        print(f"[build] Copying {len(source_packages)} packages for '{comp_name}' → {ws_src}")
        if os.path.exists(ws_src):
            shutil.rmtree(ws_src)
        os.makedirs(ws_src, exist_ok=True)

        # Copy each package
        for abs_source, pkg_name in source_packages:
            dest = os.path.join(ws_src, pkg_name)
            print(f"  - copying {pkg_name}")
            shutil.copytree(abs_source, dest, symlinks=False)

        # load any post-install hooks
        # (your Component model should expose postinstall as a List[str])
        post_hooks = comp.postinstall or []

        # assemble the build command
        cmds = [
            "source /opt/ros/$ROS_DISTRO/setup.bash",
            "colcon build --symlink-install --install-base install"
        ]
        for p in post_hooks:
            cmds.append(
                "source /opt/ros/$ROS_DISTRO/setup.bash && "
                "source /ros_ws/install/setup.bash && "
                f"{p}"
            )
        full_cmd = " && ".join(cmds)

        print(f"[build] [{comp_name}] Running build container:")
        print(f"         {full_cmd}")

        # run the unified builder image (created in stage)
        image = comp.image_tag(cfg)
        docker.run(
            image       = image,
            command     = ["bash", "-lc", full_cmd],
            remove      = True,
            tty         = True,
            workdir     = "/ros_ws",
            envs        = {"ROS_DISTRO": cfg.ros_distro},
            volumes     = [(os.path.abspath(ws_root), "/ros_ws", "rw")],
        )

        print(f"[build] '{comp_name}' done; install at {ws_root}/install")

if __name__ == "__main__":
    # usage: build.py [project_root] [--component name]
    args = sys.argv[1:]
    pr   = args[0] if len(args) >= 1 else "."
    comp = args[1] if len(args) == 2 else None
    build_main(pr, comp)
