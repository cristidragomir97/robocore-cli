#!/usr/bin/env python3
import os
import sys
import argparse
import traceback
from colorama import init as colorama_init, Fore

from commands.init    import init_main
from commands.stage   import stage_main
from commands.build   import build_main
from commands.deploy  import deploy_main
from commands.shell   import shell_main

def create_parser():
    colorama_init(autoreset=True)
    p = argparse.ArgumentParser(
        prog="robocore-cli",
        description="robocore-cli: build & deploy ROS2 in Docker"
    )
    p.add_argument('-p','--project-root', dest='project_root',
                   help='Path to project root')
    p.add_argument('project_root_pos', nargs='?',
                   help=argparse.SUPPRESS)

    sp = p.add_subparsers(dest='command', required=True)

    # init
    pi = sp.add_parser('init', help='Bootstrap a new project')
    pi.set_defaults(func=lambda args: init_main(args.project_root))

    # stage (formerly prep)
    ps = sp.add_parser('stage', help='Generate multi-stage Dockerfiles & Compose')
    ps.add_argument('-c','--component', default=None,
                    help='Only stage this single component')
    ps.set_defaults(func=lambda args: stage_main(
        project_root=args.project_root,
        component=args.component
    ))

    # build
    pb = sp.add_parser('build', help='Compile workspaces in Docker')
    pb.add_argument('-c','--component', default=None,
                    help='Only build this single component')
    pb.set_defaults(func=lambda args: build_main(
        project_root=args.project_root,
        component=args.component
    ))

    # deploy
    pd = sp.add_parser('deploy', help='Rsync builds & launch containers')
    pd.add_argument('--simulate', action='store_true',
                    help='Run only simulate:true services locally')
    pd.set_defaults(func=lambda args: deploy_main(
        project_root=args.project_root,
        simulate=args.simulate
    ))

    # shell
    pc = sp.add_parser(
            'shell',
            help='Open an interactive ROS2 shell for <component|common|path>'
        )
    
    pc.add_argument(
            'target',
            help='Either a component name, "common", or a path to a ros_ws folder'
        )
    
    pc.set_defaults(func=lambda args: shell_main(
            target       = args.target,
            project_root = args.project_root
        ))

    return p

def main():
    parser = create_parser()
    args   = parser.parse_args()

    pr = args.project_root or args.project_root_pos
    if not pr and args.command!='init':
        parser.print_usage()
        sys.exit("[ERROR] project root must be set via -p or positional")
    args.project_root = os.path.abspath(pr) if pr else None

    try:
        args.func(args)
    except Exception:
        print(Fore.RED + "[ERROR] Unhandled exception:", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

if __name__=='__main__':
    main()
