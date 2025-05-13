#!/usr/bin/env python3
import sys
import argparse
from colorama import init as colorama_init, Fore

from prep import prep_main
from build import build_main
from deploy import deploy_main

class ExitException(Exception):
    """Signal a clean exit with a specific return code."""
    def __init__(self, msg, code=1):
        super().__init__(msg)
        self.code = code

def main():
    # Initialize Colorama (auto‐reset after each print)
    colorama_init(autoreset=True)

    parser = argparse.ArgumentParser(
        prog="rosdock",
        description="rosdock: build & deploy ROS2 in Docker"
    )
    parser.add_argument(
        '-p', '--project-root',
        default='.',
        help='Path to the root of your rosdock solution'
    )

    subparsers = parser.add_subparsers(dest='command', required=True)

    # prep subcommand
    prep_parser = subparsers.add_parser(
        'prep',
        help='Render Dockerfiles, build & push images, and generate docker-compose.yml'
    )
    prep_parser.add_argument(
        '-c', '--component',
        default=None,
        help='Only prep this single component'
    )

    # build subcommand
    build_parser = subparsers.add_parser(
        'build',
        help='Compile ROS packages into build/<component>/ros_ws/install'
    )
    build_parser.add_argument(
        '-c', '--component',
        default=None,
        help='Only build this single component'
    )

    # deploy subcommand
    deploy_parser = subparsers.add_parser(
        'deploy',
        help='Rsync build artifacts to robots and bring up Docker Compose (streaming logs)'
    )

    args = parser.parse_args()

    pr = args.project_root

    try:
        if args.command == 'prep':
            print(Fore.GREEN + "[STEP] Running prep…")
            prep_main(project_root=pr, component=args.component)
            print(Fore.GREEN + "[OK] prep complete")

        elif args.command == 'build':
            print(Fore.GREEN + "[STEP] Running build…")
            build_main(project_root=pr, component=args.component)
            print(Fore.GREEN + "[OK] build complete")

        elif args.command == 'deploy':
            print(Fore.GREEN + "[STEP] Running deploy…")
            deploy_main(project_root=pr)
            print(Fore.GREEN + "[OK] deploy complete")

    except ExitException as e:
        print(Fore.RED + f"[ERROR] {e}", file=sys.stderr)
        sys.exit(e.code)
    except Exception as e:
        print(Fore.RED + f"[UNEXPECTED ERROR] {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
