# main.py

import argparse
import os
import sys

from rich.console import Console

from parser import Parser
from environment_manager import EnvironmentManager
from host_manager import HostManager

console = Console()

def parse_args():
    parser = argparse.ArgumentParser(description="Refactored CLI Tool with Single Endpoint in Brick")

    parser.add_argument(
        "--solution-dir",
        "-d",
        type=str,
        default=".",
        help="Path to the base solution directory."
    )
    parser.add_argument(
        "--pull",
        action="store_true",
        help="Pull new changes for existing Git repos (if any)."
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove Docker images on each host before building."
    )
    return parser.parse_args()

def clean_docker_images(hosts):
    """
    Example function to remove all Docker images on each host via SSH.
    """
    for host in hosts:
        host_mgr = HostManager(host)
        console.print(f"[bold yellow]Cleaning Docker images on '{host.name}'[/bold yellow]")
        # Just an example: remove all images
        host_mgr.run_ssh_command("docker rmi --force $(docker images -q)")

def main():
    args = parse_args()
    solution_dir = os.path.abspath(args.solution_dir)

    # 1) Parse config (hosts + bricks) from your YAML files
    config_parser = Parser(solution_dir)
    hosts = config_parser.hosts
    bricks = config_parser.bricks
    settings = config_parser.settings  # e.g. {"ros_version": "noetic", ...}

    # 2) Optionally clean Docker images first
    if args.clean:
        clean_docker_images(hosts)

    # 3) Create EnvironmentManager for local tasks
    env_mgr = EnvironmentManager(solution_dir)

    # 4) For each host, build the bricks
    for host in hosts:
        host_mgr = HostManager(host)
        bricks_for_host = [b for b in bricks if b.host == host.name]

        for brick in bricks_for_host:
            # a) Create local directories
            brick_dir = env_mgr.get_brick_dir(host.name, brick.id)
            ros_ws_dir = os.path.join(brick_dir, "ros_ws")

            # b) Prepare the source code (Git clone/pull or copy local)
            env_mgr.prepare_source(brick, ros_ws_dir, pull=args.pull)

            # c) Detect dependencies -> choose base image
            deps = env_mgr.detect_dependencies(ros_ws_dir)
            ros_version = settings.get("ros_version", "noetic")
            base_image = env_mgr.choose_base_image(deps, ros_version, registry="")

            # d) Generate Dockerfile
            dockerfile_path = os.path.join(brick_dir, "Dockerfile")
            env_mgr.generate_dockerfile(
                brick=brick,
                ros_distro=ros_version,
                source_dir=ros_ws_dir,
                dockerfile_path=dockerfile_path,
                base_image=base_image
            )

            # e) Build image on remote host
            tag = f"rc-{ros_version}-{brick.id}-{host.arch}"
            host_mgr.build_image(
                build_path=brick_dir,
                dockerfile_name="Dockerfile",
                tag=tag
            )
            console.print(f"[green]Built image '{tag}' for brick '{brick.id}' on '{host.name}'.[/green]")

    console.print("[bold magenta]All bricks processed successfully![/bold magenta]")

if __name__ == "__main__":
    main()
