#!/usr/bin/env python3
import os
import sys
import shutil
import yaml
import platform
from typing import Optional, List, Tuple

from python_on_whales import DockerClient
from core.config   import Config
from core.models   import Component, Host

def get_host_arch() -> str:
    """Get the current machine's architecture in Docker format."""
    machine = platform.machine().lower()
    if machine in ('x86_64', 'amd64'):
        return 'amd64'
    elif machine in ('aarch64', 'arm64'):
        return 'arm64'
    elif machine.startswith('arm'):
        return 'armv7'
    return machine

def get_host(hosts_map: dict, comp: Component, cfg: Config) -> Host:
    """Get the host for a component based on runs_on."""
    if not comp.runs_on:
        sys.exit(f"[build] ERROR: component '{comp.name}' missing 'runs_on'")
    host = hosts_map.get(comp.runs_on)
    if not host:
        sys.exit(f"[build] ERROR: runs_on '{comp.runs_on}' not defined")
    return host

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

def build_main(project_root: str, component: Optional[str] = None, config_file: str = 'config.yaml'):
    """
    For each component that has local source packages:
      1) Copy sources → build/<comp>/ros_ws/src
      2) Run any postinstall hooks
      3) Invoke the builder image (already staged) to colcon build → install
    """
    # 1) chdir into project
    project_root = os.path.abspath(project_root)
    os.chdir(project_root)

    # 2) load config & pick components
    cfg   = Config.load(project_root, config_file=config_file)
    docker = DockerClient()
    comps = cfg.filter_components(name=component)
    hosts_map = {h.name: h for h in cfg.hosts}
    host_arch = get_host_arch()
    if not comps:
        print(f"[build] No components to build (filter={component})")
        return

    build_root = os.path.abspath(cfg.build_dir)
    os.makedirs(build_root, exist_ok=True)

    # 3) loop
    for comp in comps:
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

        # Determine target platform for potential emulation
        host = get_host(hosts_map, comp, cfg)
        target_arch = host.arch
        target_platform = f"linux/{target_arch}"

        if target_arch != host_arch:
            print(f"[build] [{comp_name}] Cross-compiling: {host_arch} → {target_arch} (using emulation)")

        print(f"[build] [{comp_name}] Running build container:")
        print(f"         {full_cmd}")

        # run the unified builder image (created in stage)
        image = comp.image_tag(cfg)

        # Pull image with correct platform if cross-compiling
        if target_arch != host_arch:
            print(f"[build] [{comp_name}] Pulling image for {target_platform}...")
            docker.image.pull(image, platform=target_platform)

        docker.run(
            image       = image,
            command     = ["bash", "-lc", full_cmd],
            remove      = True,
            tty         = True,
            workdir     = "/ros_ws",
            envs        = {"ROS_DISTRO": cfg.ros_distro},
            volumes     = [(os.path.abspath(ws_root), "/ros_ws", "rw")],
            platform    = target_platform,
        )

        print(f"[build] '{comp_name}' done; install at {ws_root}/install")

if __name__ == "__main__":
    # usage: build.py [project_root] [--component name]
    args = sys.argv[1:]
    pr   = args[0] if len(args) >= 1 else "."
    comp = args[1] if len(args) == 2 else None
    build_main(pr, comp)
