#!/usr/bin/env python3
import sys
import os
import argparse, traceback
from colorama import init as colorama_init, Fore

from commands.init   import init_main
from commands.prep   import prep_main
from commands.build  import build_main
from commands.deploy import deploy_main
from commands.shell  import shell_main


def register_init(sp):
    p = sp.add_parser('init', help='Bootstrap a new rosdock project')
    p.set_defaults(func=lambda args: init_main(args.project_root))

def register_prep(sp):
    p = sp.add_parser('prep', help='Generate Dockerfiles & Compose')
    p.add_argument('-c','--component', default=None,
                   help='Only prep this single component')
    p.set_defaults(func=lambda args: prep_main(
        project_root=args.project_root,
        component=args.component
    ))

def register_build(sp):
    p = sp.add_parser('build', help='Compile workspaces in Docker')
    p.add_argument('-c','--component', default=None,
                   help='Only build this single component')
    p.set_defaults(func=lambda args: build_main(
        project_root=args.project_root,
        component=args.component
    ))

def register_deploy(sp):
    p = sp.add_parser('deploy', help='Rsync builds & launch containers')
    p.set_defaults(func=lambda args: deploy_main(
        project_root=args.project_root
    ))

def register_shell(sp):
    p = sp.add_parser('shell', help='Open an interactive ROS2 shell')
    p.add_argument('path', help='Path to your ros_ws folder')
    p.set_defaults(func=lambda args: shell.shell_main(path=args.path))

def create_parser():
    colorama_init(autoreset=True)
    parser = argparse.ArgumentParser(
        prog="rosdock",
        description="rosdock: build & deploy ROS2 in Docker"
    )
    parser.add_argument('-p','--project-root', dest='project_root',
                        default=None, help='Path to project root')
    parser.add_argument('project_root_pos', nargs='?',
                        default=None, help=argparse.SUPPRESS)

    sp = parser.add_subparsers(dest='command', required=True)
    register_init(sp)
    register_prep(sp)
    register_build(sp)
    register_deploy(sp)
    register_shell(sp)
    return parser

def main():
    parser = create_parser()
    args   = parser.parse_args()

    pr = args.project_root or args.project_root_pos
    if not pr:
        parser.print_usage()
        sys.exit("[ERROR] project root must be set via -p or as first arg")
    args.project_root = os.path.abspath(pr)

    if args.command != 'init' and not os.path.isdir(args.project_root):
        sys.exit(f"[ERROR] project root '{args.project_root}' does not exist")

    try:
        args.func(args)
    except Exception:
        import traceback
        print(Fore.RED + "[ERROR] Unhandled exception:", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

if __name__=='__main__':
    main()