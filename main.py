#!/usr/bin/env python3

import argparse
import os
import json
import sys
from typing import Dict, List
from entities.host import Host
from entities.brick import Brick 
from rich.console import Console
from rich.table import Table

# Import your existing modules (adjust paths as needed)
from parser import Parser
from managers.environment import EnvironmentManager
from managers.host import HostManager
from managers.run import RunManager

console = Console()

def parse_args():
    parser = argparse.ArgumentParser(description="CLI Tool with Subcommands: build, run")

    subparsers = parser.add_subparsers(
        title="commands",
        dest="command",
        required=True,
        help="Available subcommands"
    )

    # --- Subcommand: build ---
    build_parser = subparsers.add_parser("build", help="buildhronize and build images.")
    build_parser.add_argument(
        "--solution-dir",
        "-d",
        type=str,
        default=".",
        help="Path to the base solution directory."
    )
    build_parser.add_argument(
        "--pull",
        action="store_true",
        help="Pull new changes for existing Git repos (if any)."
    )
    build_parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove Docker images on each host before building."
    )

    # --- Subcommand: run ---
    run_parser = subparsers.add_parser("run", help="Run images or containers (future feature).")
    run_parser.add_argument(
        "--solution-dir",
        "-d",
        type=str,
        default=".",
        help="Path to the base solution directory."
    )
    # Add more arguments for 'run' if needed
    ps_parser = subparsers.add_parser("ps", help="Run images or containers (future feature).")
    ps_parser.add_argument(
        "--solution-dir",
        "-d",
        type=str,
        default=".",
        help="Path to the base solution directory."
    )

    return parser.parse_args()

def clean_docker_images(hosts):
    """
    Example function to remove all Docker images on each host via SSH or Docker API.
    """
    for host in hosts:
        host_mgr = HostManager(host)
        console.print(f"[bold yellow]Cleaning Docker images on '{host.name}'[/bold yellow]")
        host_mgr.run_ssh_command("docker rmi --force $(docker images -q)")

def cmd_build(hosts, bricks, settings, solution_dir: str, pull: bool, clean: bool):
    """
    Implements the 'build' subcommand:
      1. Parse config
      2. Optionally clean Docker images
      3. Build images for each brick on each host
      4. Save the built tags into a JSON file
    """
    # 1) Parse config


    # 2) Possibly clean Docker images first
    if clean:
        clean_docker_images(hosts)

    # 3) Create environment manager for local tasks
    env_mgr = EnvironmentManager(solution_dir)

    # We'll store built images in a dict: {brick_id: docker_tag}
    # or you can store them per-host if desired.
    built_images: Dict[str, str] = {}

    # For each host, build the bricks
    for host in hosts:
        host_mgr = HostManager(host)
        bricks_for_host = [b for b in bricks if b.host == host.name]

        for brick in bricks_for_host:
            # a) Create local folder structure
            brick_dir = env_mgr.get_brick_dir(host.name, brick.id)
            ros_ws_dir = os.path.join(brick_dir, "ros_ws")

            # b) Prepare the source code
            env_mgr.prepare_source(brick, ros_ws_dir, pull=pull)

            # c) Detect dependencies, choose base image
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

            # e) Build the Docker image on the remote host
            tag = f"rc-{ros_version}-{brick.id}-{host.arch}"
            console.print(f"[blue]Building image '{tag}' for brick '{brick.id}' on host '{host.name}'[/blue]")
            for output in host_mgr.build_image(
                build_path=brick_dir,
                dockerfile_name="Dockerfile",
                tag=tag
            ):
                console.print(f"{brick.id}", output)

            # f) Store the built image tag for reference
            built_images[brick.id] = tag

    # 4) Save built image info to a JSON file
    build_dir = env_mgr.get_build_dir()
    images_file = os.path.join(build_dir, "built_images.json")
    with open(images_file, "w") as f:
        json.dump(built_images, f, indent=2)
    console.print(f"[green]Image tags saved to {images_file}[/green]")

def cmd_run(solution_dir: str):
    """
    'run' command:
      1. Parse config to get bricks/hosts
      2. Load built_images.json
      3. For each brick that has a 'run' or 'launch', start container
    """
    # 1) Parse config
    config_parser = Parser(solution_dir)
    hosts = config_parser.hosts
    bricks = config_parser.bricks

    # 2) Load built_images.json
    build_dir = os.path.join(os.path.abspath(solution_dir), "build")
    images_file = os.path.join(build_dir, "built_images.json")
    if not os.path.isfile(images_file):
        console.print(f"[red]No built_images.json found at {images_file}. Did you run 'sync' first?[/red]")
        sys.exit(1)

    with open(images_file, "r") as f:
        built_images = json.load(f)  # {brick_id: docker_tag}

    # 3) For each host & its bricks, run containers if package.run or package.launch is set
    for host in hosts:
        host_mgr = HostManager(host)
        run_mgr = RunManager(host_mgr, built_images)

        bricks_for_host = [b for b in bricks if b.host == host.name]
        for brick in bricks_for_host:
            if not brick.package:
                continue

            # Decide if we have something to run
            if brick.package.run or brick.package.launch:
                run_mgr.run_brick_container(brick)
            else:
                console.print(f"[grey]Brick '{brick.id}' has no run/launch field, skipping.[/grey]")


def cmd_ps(hosts):
    for host in hosts:
        host_mgr = HostManager(host)
        running_containers = host_mgr.list_running_containers()
        
        console.print(f"[bold cyan] Running containers on {host.name}[/bold cyan]")
        table = Table(show_header=True, header_style="bold cyan")
        table.add_column("Container ID", style="bold", justify="left")
        table.add_column("Name", style="green", justify="left")
        table.add_column("Image", style="magenta", justify="left")
        table.add_column("Status", style="yellow", justify="left")

        for container in running_containers:
            table.add_row(
                container.id,
                container.name,
                ", ".join(container.image.tags) if container.image.tags else "<none>",
                container.status
            )

        console.print(table)


def main():
    args = parse_args()
    solution_dir = os.path.abspath(args.solution_dir)
    config_parser = Parser(solution_dir)
    hosts = config_parser.hosts
    bricks = config_parser.bricks
    settings = config_parser.settings  # e.g., {"ros_version": "humble", ...}

    if args.command == "build":
        cmd_build(
            hosts, 
            bricks, 
            settings,
            solution_dir= solution_dir,
            pull=getattr(args, "pull", False),
            clean=getattr(args, "clean", False),
        )
    elif args.command == "run":
        cmd_run(
            solution_dir=os.path.abspath(args.solution_dir)
        )
    elif args.command == "ps":
        cmd_ps(hosts)
    elif args.command == "watch":
        pass
    else:
        console.print("[red]Unknown command[/red]")
        sys.exit(1)

if __name__ == "__main__":
    main()
