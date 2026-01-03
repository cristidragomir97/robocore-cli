#!/usr/bin/env python3
"""
RoboStack environment activation command.
Opens a shell with robostack environment, DDS superclient config, and manager IP.
"""
import os
import sys
import subprocess
import tempfile
from typing import Optional
from colorama import Fore

from core.config import Config
from core.renderer import TemplateRenderer
from core.exceptions import RobocoreError


def detect_package_manager() -> Optional[str]:
    """
    Detect which package manager is available (micromamba, mamba, or pixi).
    Returns the name of the first one found, or None if none are available.
    """
    managers = ['micromamba', 'mamba', 'pixi']

    for manager in managers:
        try:
            # For micromamba, check MAMBA_EXE environment variable first
            if manager == 'micromamba' and 'MAMBA_EXE' in os.environ:
                mamba_exe = os.environ['MAMBA_EXE']
                result = subprocess.run(
                    [mamba_exe, '--version'],
                    capture_output=True,
                    text=True,
                    check=False
                )
                if result.returncode == 0:
                    return manager

            # Standard detection for all managers
            result = subprocess.run(
                [manager, '--version'],
                capture_output=True,
                text=True,
                check=False
            )
            if result.returncode == 0:
                return manager
        except FileNotFoundError:
            continue

    return None


def check_environment_exists(manager: str, env_name: str) -> bool:
    """Check if an environment exists using the detected package manager."""
    try:
        # pixi uses a different command structure
        if manager == 'pixi':
            result = subprocess.run(
                ['pixi', 'info'],
                capture_output=True,
                text=True,
                check=False,
                cwd=os.getcwd()
            )
            # For pixi, we just check if pixi.toml exists and is valid
            return result.returncode == 0
        else:
            # micromamba and mamba use the same env list command
            # For micromamba, use MAMBA_EXE if available
            cmd = manager
            if manager == 'micromamba' and 'MAMBA_EXE' in os.environ:
                cmd = os.environ['MAMBA_EXE']

            result = subprocess.run(
                [cmd, 'env', 'list'],
                capture_output=True,
                text=True,
                check=True
            )
            # Parse output to find environment name
            for line in result.stdout.split('\n'):
                if line.strip().startswith(env_name):
                    return True
            return False
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


def get_manager_host(cfg: Config):
    """Get the manager host from config."""
    for host in cfg.hosts:
        if host.manager:
            return host
    return None


def verify_package_manager(manager: str) -> bool:
    """Verify that a specific package manager is installed."""
    try:
        # For micromamba, check MAMBA_EXE environment variable first
        if manager == 'micromamba' and 'MAMBA_EXE' in os.environ:
            mamba_exe = os.environ['MAMBA_EXE']
            result = subprocess.run(
                [mamba_exe, '--version'],
                capture_output=True,
                text=True,
                check=False
            )
            if result.returncode == 0:
                return True

        # Standard verification
        result = subprocess.run(
            [manager, '--version'],
            capture_output=True,
            text=True,
            check=False
        )
        return result.returncode == 0
    except FileNotFoundError:
        return False


def stack_main(project_root: Optional[str] = None, env_name: str = "ros_env", package_manager: Optional[str] = None, config_file: str = 'config.yaml'):
    """
    Activate robostack environment with DDS configuration.

    Args:
        project_root: Path to project root
        config_file: Name or path to configuration file (default: 'config.yaml')
        env_name: Name of the robostack environment (default: "ros_env")
        package_manager: Explicitly specified package manager (micromamba, mamba, or pixi)
        config_file: Name or path to configuration file (default: 'config.yaml')
    """
    print(Fore.CYAN + "[stack] Initializing robostack environment...")

    # 1) Detect or verify package manager
    if package_manager:
        # User specified a package manager - verify it's available
        if not verify_package_manager(package_manager):
            print(Fore.RED + f"[stack] ERROR: Specified package manager '{package_manager}' not found or not working")
            print(Fore.YELLOW + f"[stack] Install {package_manager} or omit the --{package_manager} flag to auto-detect")
            sys.exit(1)
        manager = package_manager
        print(Fore.GREEN + f"[stack] ✓ {manager} found (user-specified)")
    else:
        # Auto-detect package manager
        manager = detect_package_manager()
        if not manager:
            print(Fore.RED + "[stack] ERROR: No package manager found (micromamba, mamba, or pixi)")
            print(Fore.YELLOW + "[stack] Install one of the following:")
            print(Fore.YELLOW + "  - micromamba: https://mamba.readthedocs.io/en/latest/installation/micromamba-installation.html")
            print(Fore.YELLOW + "  - mamba: https://mamba.readthedocs.io/")
            print(Fore.YELLOW + "  - pixi: https://pixi.sh/")
            sys.exit(1)
        print(Fore.GREEN + f"[stack] ✓ {manager} found (auto-detected)")

    # 2) Check if environment exists (pixi handles this differently)
    if manager == 'pixi':
        # For pixi, check if we're in a pixi project
        if not os.path.exists('pixi.toml') and not os.path.exists('pyproject.toml'):
            print(Fore.RED + "[stack] ERROR: Not in a pixi project (no pixi.toml or pyproject.toml found)")
            print(Fore.YELLOW + "[stack] Initialize a pixi project with:")
            print(Fore.YELLOW + "  pixi init")
            print(Fore.YELLOW + "  pixi add ros-humble-desktop")
            sys.exit(1)
        print(Fore.GREEN + "[stack] ✓ pixi project found")
    else:
        # For mamba/micromamba, check if environment exists
        if not check_environment_exists(manager, env_name):
            print(Fore.RED + f"[stack] ERROR: {manager} environment '{env_name}' not found")
            print(Fore.YELLOW + f"[stack] Available environments:")
            subprocess.run([manager, 'env', 'list'])
            print(Fore.YELLOW + f"\n[stack] Create the environment with:")
            print(Fore.YELLOW + f"  {manager} create -n {env_name} ros-humble python=3.10")
            print(Fore.YELLOW + f"  {manager} activate {env_name}")
            print(Fore.YELLOW + f"  {manager} install -c robostack-staging ros-humble-desktop")
            sys.exit(1)

        print(Fore.GREEN + f"[stack] ✓ environment '{env_name}' found")

    # 3) Load project config
    pr = os.path.abspath(project_root or ".")
    try:
        cfg = Config.load(pr, config_file=config_file)
    except FileNotFoundError:
        print(Fore.RED + f"[stack] ERROR: Configuration file '{config_file}' not found in {pr}")
        print(Fore.YELLOW + "[stack] Run 'robocore-cli init' to create a project first")
        sys.exit(1)
    except Exception as e:
        print(Fore.RED + f"[stack] ERROR: Failed to load config: {e}")
        sys.exit(1)

    print(Fore.GREEN + "[stack] ✓ config loaded")

    # 4) Get manager host
    manager_host = get_manager_host(cfg)
    if not manager_host:
        print(Fore.RED + "[stack] ERROR: No manager host found in config.yaml")
        print(Fore.YELLOW + "[stack] Add 'manager: true' to one of your hosts")
        sys.exit(1)

    manager_ip = manager_host.effective_dds_ip
    print(Fore.GREEN + f"[stack] ✓ manager host: {manager_host.name} ({manager_ip})")

    # 5) Check for persistent superclient.xml from stage command
    robocore_dir = os.path.join(pr, '.robocore')
    superclient_path = os.path.join(robocore_dir, 'superclient.xml')

    if not os.path.exists(superclient_path):
        print(Fore.YELLOW + f"[stack] WARNING: superclient.xml not found at {superclient_path}")
        print(Fore.YELLOW + f"[stack] Run 'robocore-cli stage' to generate it")
        print(Fore.YELLOW + f"[stack] Continuing with ROS_DISCOVERY_SERVER only...")
        superclient_path = None
    else:
        print(Fore.GREEN + f"[stack] ✓ Using superclient.xml from {superclient_path}")

    # 6) Create activation script and launch shell
    print(Fore.CYAN + "\n[stack] Launching shell with robostack environment...")
    if manager != 'pixi':
        print(Fore.CYAN + f"[stack] Environment: {env_name}")
    print(Fore.CYAN + f"[stack] Package Manager: {manager}")
    print(Fore.CYAN + f"[stack] ROS_DOMAIN_ID: {cfg.ros_domain_id}")
    if superclient_path:
        print(Fore.CYAN + f"[stack] FASTRTPS_DEFAULT_PROFILES_FILE: {superclient_path}")
        print(Fore.CYAN + f"[stack] DDS Server: {manager_ip}:11811 (via superclient.xml)")
    else:
        print(Fore.CYAN + f"[stack] ROS_DISCOVERY_SERVER: {manager_ip}:11811")
    print(Fore.CYAN + "\n[stack] Type 'exit' to close the robostack shell\n")

    # Create activation script based on detected package manager
    # Build the DDS configuration lines
    if superclient_path:
        # When using superclient.xml, don't set ROS_DISCOVERY_SERVER as it can conflict
        dds_config = f"""export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE={superclient_path}"""
    else:
        # Fallback to ROS_DISCOVERY_SERVER if no superclient.xml
        dds_config = f"""export ROS_DISCOVERY_SERVER={manager_ip}:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"""

    # Use a temporary directory for activation scripts
    with tempfile.TemporaryDirectory() as tmpdir:
        if manager == 'pixi':
            # Pixi activation
            activation_script = f"""
# Activate pixi environment
eval "$(pixi shell-hook)"

# Set ROS and DDS environment variables
export ROS_DOMAIN_ID={cfg.ros_domain_id}
export ROS_DISTRO={cfg.ros_distro}
{dds_config}

# Stop ROS 2 daemon to avoid DDS configuration conflicts
echo "Stopping ROS 2 daemon (to avoid discovery conflicts)..."
ros2 daemon stop 2>/dev/null || true

# Print info
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  RoboStack Environment Active (pixi)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  ROS Distribution: {cfg.ros_distro}"
echo "  ROS Domain ID:    {cfg.ros_domain_id}"
echo "  DDS Server:       {manager_ip}:11811"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
"""
        elif manager == 'micromamba':
            # Micromamba activation - use MAMBA_EXE if available
            mamba_exe = os.environ.get('MAMBA_EXE', 'micromamba')
            mamba_root = os.environ.get('MAMBA_ROOT_PREFIX', '$HOME/micromamba')

            activation_script = f"""
# Initialize micromamba
export MAMBA_EXE='{mamba_exe}'
export MAMBA_ROOT_PREFIX='{mamba_root}'
eval "$("$MAMBA_EXE" shell hook --shell bash --root-prefix "$MAMBA_ROOT_PREFIX" 2> /dev/null)"

# Activate the environment
micromamba activate {env_name}

# Set ROS and DDS environment variables
export ROS_DOMAIN_ID={cfg.ros_domain_id}
export ROS_DISTRO={cfg.ros_distro}
{dds_config}

# Stop ROS 2 daemon to avoid DDS configuration conflicts
echo "Stopping ROS 2 daemon (to avoid discovery conflicts)..."
ros2 daemon stop 2>/dev/null || true

# Print info
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  RoboStack Environment Active (micromamba)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Environment:      {env_name}"
echo "  ROS Distribution: {cfg.ros_distro}"
echo "  ROS Domain ID:    {cfg.ros_domain_id}"
echo "  DDS Server:       {manager_ip}:11811"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
"""
        else:  # mamba
            # Mamba/conda activation
            activation_script = f"""
# Activate mamba/conda
if [ -f "$HOME/mambaforge/etc/profile.d/conda.sh" ]; then
    source "$HOME/mambaforge/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniforge3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniforge3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    source "/opt/conda/etc/profile.d/conda.sh"
fi

if [ -f "$HOME/mambaforge/etc/profile.d/mamba.sh" ]; then
    source "$HOME/mambaforge/etc/profile.d/mamba.sh"
elif [ -f "$HOME/miniforge3/etc/profile.d/mamba.sh" ]; then
    source "$HOME/miniforge3/etc/profile.d/mamba.sh"
fi

# Activate the environment
mamba activate {env_name}

# Set ROS and DDS environment variables
export ROS_DOMAIN_ID={cfg.ros_domain_id}
export ROS_DISTRO={cfg.ros_distro}
{dds_config}

# Stop ROS 2 daemon to avoid DDS configuration conflicts
echo "Stopping ROS 2 daemon (to avoid discovery conflicts)..."
ros2 daemon stop 2>/dev/null || true

# Print info
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  RoboStack Environment Active (mamba)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Environment:      {env_name}"
echo "  ROS Distribution: {cfg.ros_distro}"
echo "  ROS Domain ID:    {cfg.ros_domain_id}"
echo "  DDS Server:       {manager_ip}:11811"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
"""

        # Write activation script to a temporary file
        script_path = os.path.join(tmpdir, 'activate.sh')
        with open(script_path, 'w') as f:
            f.write(activation_script)

        # Launch shell with the activation script
        shell = os.environ.get('SHELL', '/bin/bash')
        shell_name = os.path.basename(shell)

        try:
            if shell_name == 'zsh':
                # For zsh, create a ZDOTDIR with .zshrc
                zdotdir = os.path.join(tmpdir, 'zdotdir')
                os.makedirs(zdotdir, exist_ok=True)
                zshrc_path = os.path.join(zdotdir, '.zshrc')

                # Create .zshrc that sources the original and our script
                with open(zshrc_path, 'w') as f:
                    f.write(f"""
# Source original .zshrc if it exists
if [ -f "$HOME/.zshrc" ]; then
    source "$HOME/.zshrc"
fi

# Source activation script
source {script_path}
""")

                # Launch zsh with custom ZDOTDIR
                env = os.environ.copy()
                env['ZDOTDIR'] = zdotdir
                subprocess.run(
                    [shell, '-i'],
                    env=env,
                    check=False
                )
            else:
                # For bash and other shells, use --rcfile if supported
                subprocess.run(
                    [shell, '--rcfile', script_path, '-i'],
                    check=False
                )
        except KeyboardInterrupt:
            print(Fore.YELLOW + "\n[stack] Shell closed")

        print(Fore.GREEN + "\n[stack] Robostack shell closed")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 -m commands.stack <project_root> [env_name]", file=sys.stderr)
        sys.exit(1)

    env = sys.argv[2] if len(sys.argv) > 2 else "ros_env"
    stack_main(project_root=sys.argv[1], env_name=env)
