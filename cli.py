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
from commands.prepare_base import prepare_base_main
from commands.viz import viz_main
from commands.clean import clean_main
from commands.validate import validate_main
from commands.stack import stack_main
from core.exceptions import RobocoreError, ConfigurationError, ValidationError
from python_on_whales.exceptions import DockerException
import subprocess

def create_parser():
    colorama_init(autoreset=True)
    p = argparse.ArgumentParser(
        prog="robocore-cli",
        description="robocore-cli: build & deploy ROS2 in Docker"
    )
    p.add_argument('-p','--project-root', dest='project_root',
                   help='Path to project root')
    p.add_argument('-f','--config', dest='config_file', default='config.yaml',
                   help='Path to configuration file (default: config.yaml)')
    p.add_argument('project_root_pos', nargs='?',
                   help=argparse.SUPPRESS)

    sp = p.add_subparsers(dest='command', required=True)


    pv = sp.add_parser('viz', help='Launch local ROS2 GUI desktop (RViz, rqt) in a container')
    pv.set_defaults(func=lambda args: viz_main(args.project_root, config_file=args.config_file))

    pb = sp.add_parser('prepare-base', help='Build base ROS image')
    pb.set_defaults(func=lambda args: prepare_base_main(args.project_root, config_file=args.config_file))

    # init
    pi = sp.add_parser('init', help='Bootstrap a new project')
    pi.set_defaults(func=lambda args: init_main(args.project_root, config_file=args.config_file))

    # stage (formerly prep)
    ps = sp.add_parser('stage', help='Generate multi-stage Dockerfiles & Compose')
    ps.add_argument('-c','--component', default=None,
                    help='Only stage this single component')
    ps.add_argument('--refresh', action='store_true', help='Only re-render docker-compose.yml')
    ps.add_argument('--force-base', action='store_true', help='Force rebuild of base image')
    ps.set_defaults(func=lambda args: stage_main(
        project_root=args.project_root,
        component=args.component,
        refresh=args.refresh,
        force_base=args.force_base,
        config_file=args.config_file
    ))

    # build
    pb = sp.add_parser('build', help='Compile workspaces in Docker')
    pb.add_argument('-c','--component', default=None,
                    help='Only build this single component')
    pb.set_defaults(func=lambda args: build_main(
        project_root=args.project_root,
        component=args.component,
        config_file=args.config_file
    ))

    # deploy
    pd = sp.add_parser('deploy', help='Rsync builds & launch containers')
    pd.add_argument('--simulate', action='store_true',
                    help='Run only simulate:true services locally')
    pd.add_argument('--host', default=None,
                    help='Only deploy to this host (name from config file)')
    pd.set_defaults(func=lambda args: deploy_main(
        project_root=args.project_root,
        simulate=args.simulate,
        host_name=args.host,
        config_file=args.config_file
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
            project_root = args.project_root,
            config_file  = args.config_file
        ))

    # clean
    pcl = sp.add_parser('clean', help='Remove build artifacts and workspaces')
    pcl.add_argument('-r', '--remote', action='store_true',
                     help='Also clean remote hosts')
    pcl.add_argument('--local-only', action='store_true',
                     help='Clean only local artifacts (overrides -r)')
    pcl.set_defaults(func=lambda args: clean_main(
        project_root=args.project_root,
        remote=args.remote and not args.local_only,
        local=True,
        config_file=args.config_file
    ))

    # validate
    pval = sp.add_parser('validate', help='Validate configuration without executing operations')
    pval.set_defaults(func=lambda args: sys.exit(validate_main(project_root=args.project_root, config_file=args.config_file)))

    # stack
    pstack = sp.add_parser('stack', help='Activate robostack environment with DDS configuration')
    pstack.add_argument('-e', '--env', default='ros_env',
                        help='Name of the robostack environment (default: ros_env)')

    # Package manager selection (mutually exclusive)
    pm_group = pstack.add_mutually_exclusive_group()
    pm_group.add_argument('--pixi', action='store_const', const='pixi', dest='package_manager',
                          help='Use pixi package manager')
    pm_group.add_argument('--mamba', action='store_const', const='mamba', dest='package_manager',
                          help='Use mamba package manager')
    pm_group.add_argument('--micromamba', action='store_const', const='micromamba', dest='package_manager',
                          help='Use micromamba package manager')

    pstack.set_defaults(func=lambda args: stack_main(
        project_root=args.project_root,
        env_name=args.env,
        package_manager=args.package_manager,
        config_file=args.config_file
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
    except KeyboardInterrupt:
        print(Fore.YELLOW + "\n[INFO] Interrupted by user", file=sys.stderr)
        sys.exit(130)
    except (ConfigurationError, ValidationError) as e:
        print(Fore.RED + f"[ERROR] Configuration error: {e}", file=sys.stderr)
        if hasattr(e, 'context') and e.context:
            # Special handling for multiple errors
            if 'errors' in e.context:
                errors = e.context['errors']
                print(Fore.RED + f"\nFound {len(errors)} validation error(s):", file=sys.stderr)
                for i, error in enumerate(errors, 1):
                    print(Fore.RED + f"  {i}. {error}", file=sys.stderr)
            else:
                print(Fore.RED + "\nAdditional context:", file=sys.stderr)
                for key, value in e.context.items():
                    print(Fore.RED + f"  - {key}: {value}", file=sys.stderr)
        sys.exit(1)
    except RobocoreError as e:
        print(Fore.RED + f"[ERROR] {e}", file=sys.stderr)
        if hasattr(e, 'context') and e.context:
            # Special handling for multiple errors
            if 'errors' in e.context:
                errors = e.context['errors']
                print(Fore.RED + f"\nFound {len(errors)} error(s):", file=sys.stderr)
                for i, error in enumerate(errors, 1):
                    print(Fore.RED + f"  {i}. {error}", file=sys.stderr)
            else:
                print(Fore.RED + "\nAdditional context:", file=sys.stderr)
                for key, value in e.context.items():
                    print(Fore.RED + f"  - {key}: {value}", file=sys.stderr)
        sys.exit(1)
    except DockerException as e:
        print(Fore.RED + f"[ERROR] Docker command failed", file=sys.stderr)

        # Extract useful information from the exception
        if hasattr(e, 'docker_command') and e.docker_command:
            # Show simplified command (remove full paths)
            cmd = ' '.join(e.docker_command)
            cmd = cmd.replace('/usr/local/bin/docker', 'docker')
            cmd = cmd.replace('/opt/homebrew/bin/docker', 'docker')
            print(Fore.RED + f"\nCommand: {cmd}", file=sys.stderr)

        if hasattr(e, 'return_code') and e.return_code:
            print(Fore.RED + f"Exit code: {e.return_code}", file=sys.stderr)

        # The actual error output was already printed to stderr by docker
        print(Fore.YELLOW + f"\nTip: Check the output above for details", file=sys.stderr)
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(Fore.RED + f"[ERROR] Command failed", file=sys.stderr)

        # Show the command that failed
        if e.cmd:
            if isinstance(e.cmd, list):
                cmd = ' '.join(e.cmd)
            else:
                cmd = e.cmd
            print(Fore.RED + f"\nCommand: {cmd}", file=sys.stderr)

        if e.returncode:
            print(Fore.RED + f"Exit code: {e.returncode}", file=sys.stderr)

        # Show stdout/stderr if captured
        if e.stdout:
            print(Fore.YELLOW + f"\nOutput:", file=sys.stderr)
            print(e.stdout.decode() if isinstance(e.stdout, bytes) else e.stdout, file=sys.stderr)
        if e.stderr:
            print(Fore.YELLOW + f"\nError output:", file=sys.stderr)
            print(e.stderr.decode() if isinstance(e.stderr, bytes) else e.stderr, file=sys.stderr)

        sys.exit(1)
    except Exception:
        print(Fore.RED + "[ERROR] Unhandled exception:", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)

if __name__=='__main__':
    main()
