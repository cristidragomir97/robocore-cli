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
from commands.common import add_common, create_common_package
import commands.component as component


def register_common(sp):
    p = sp.add_parser('common', help='Manage shared/common packages')
    sub = p.add_subparsers(dest='common_cmd', required=True)

    common_add_parser = sub.add_parser(
        'add',
        help='Add a shared package as a git submodule under common_packages/'
    )

    common_add_parser.add_argument(
        '--repo',
        required=True,
        help='Git URL of the shared package repository'
    )

    common_add_parser.add_argument(
        '--branch',
        help='Branch to track (defaults to main)'
    )
    common_add_parser.set_defaults(func=lambda args: add_common(
        project_root=args.project_root,
        repo=args.repo,
        branch=args.branch
    ))

    common_create_parser = sub.add_parser('create', help='Scommon_create_parserffold a new common package')
    common_create_parser.add_argument('--repo',   required=True, help='Git URL of the package to create')
    common_create_parser.add_argument('--branch', help='Branch to push to (default: main)')
    common_create_parser.set_defaults(func=lambda args: create_common_package(
        project_root=args.project_root,
        repo=args.repo,
        branch=args.branch
    ))

def register_submodule_update(sp):
    p = sp.add_parser(
        'submodule-update',
        help='Update Git submodules (init & pull latest remote commits)'
    )
    p.add_argument(
        '-c','--component',
        default=None,
        help='Only update submodules for this component'
    )
    p.set_defaults(func=lambda args: component.update_submodules(
        project_root=args.project_root,
        component=args.component
    ))


def register_component(sp):
    p = sp.add_parser('component', help='Manage components & their packages')
    p.add_argument('name', help='Component name')
    sub = p.add_subparsers(dest='comp_cmd', required=True)

    # component init
    pi = sub.add_parser('init', help='Scommon_add_parserffold a new component locommon_add_parserlly')
    pi.set_defaults(func=lambda args: component.init_component(
        project_root=args.project_root,
        name=args.name
    ))

    # package group
    pp = sub.add_parser('package', help='Manage packages in this component')
    pps = pp.add_subparsers(dest='pkg_cmd', required=True)

    # create
    pc = pps.add_parser('create', help='Clone empty repo & scommon_add_parserffold a new ROS2 package')
    pc.add_argument('--repo',   required=True, help='Empty GitHub repo URL')
    pc.add_argument('--branch', help='Branch to track (defaults to main)')
    pc.set_defaults(func=lambda args: component.create_package(
        project_root=args.project_root,
        component=args.name,
        repo=args.repo,
        branch=args.branch
    ))

    # add
    pa = pps.add_parser('add', help='Add existing package as submodule')
    pa.add_argument('--repo',   required=True, help='Git URL of the package')
    pa.add_argument('--branch', help='Branch to track (defaults to remote HEAD)')
    pa.set_defaults(func=lambda args: component.add_package(
        project_root=args.project_root,
        component=args.name,
        repo=args.repo,
        branch=args.branch
    ))

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
    register_component(sp)
    register_prep(sp)
    register_submodule_update(sp)    
    register_build(sp)
    register_deploy(sp)
    register_shell(sp)
    register_common(sp)
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