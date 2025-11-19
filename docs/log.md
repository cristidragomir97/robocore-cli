# Changelog

## 2025-10-08

### Multi-Error Validation
Validation now collects and displays ALL errors at once instead of stopping at the first error.

**What changed:**
- **core/validation.py**
  - Lines 255-292: Modified `validate_config_file()` to catch Pydantic `ValidationError` and extract all field errors into a list
  - Lines 294-353: Modified `validate_source_paths_from_data()` to validate source paths from raw YAML data
  - Lines 409-467: Completely refactored `validate_all()` to collect both Pydantic validation errors and source path errors before raising
  - Now raises `ConfigurationError` with all errors in context instead of stopping at first error

- **cli.py**
  - Lines 123-151: Updated error handling to detect multiple errors and display them in a numbered list

- **commands/validate.py**
  - Lines 72-88: Updated error display to show multiple errors in numbered format

- **tests/unit/test_validation.py**
  - Line 383: Updated test to expect `ConfigurationError` instead of `SourceNotFoundError`

**Example output:**
```
[ERROR] Configuration error: Configuration validation failed

Found 2 validation error(s):
  1. : Value error, Component 'leremix_control' references unknown host 'orine'. Available hosts: orin
  2. Component 'leremix_control' source not found: ros/src/leremix_servo_managere (absolute: /Users/cdr/Work/LeRemix/ros/src/leremix_servo_managere)
```

**Impact:** Users can now fix all configuration issues in one pass instead of iteratively fixing one error at a time.

---

### Strict Schema Validation (Unknown Fields)
Added strict schema validation to reject unknown configuration fields.

**What changed:**
- **core/validation.py**
  - Added `model_config = ConfigDict(extra='forbid')` to all Pydantic models:
    - Line 24: `HostConfig`
    - Line 43: `RepositorySpec`
    - Line 64: `CommonPackageConfig`
    - Line 100: `OptimisationConfig`
    - Line 108: `ComponentConfig`
    - Line 185: `RobocoreConfig`
  - Line 201: Added `workplace_folder` field to `RobocoreConfig` schema (was missing, causing false positives)

- **tests/unit/test_validation.py**
  - Lines 427-446: Added tests for unknown field detection in components and root config

**Example output:**
```
[ERROR] Configuration error: Configuration validation failed

Found 2 validation error(s):
  1. components -> 0 -> potato: Extra inputs are not permitted
  2. workplace_folder: Extra inputs are not permitted
```

**Impact:** Typos and invalid fields in config.yaml are now caught during validation, preventing configuration drift.

---

### Added Secondary DDS IP Support for Hosts
Hosts can now have a separate IP address for DDS communication.

**What changed:**
- **core/validation.py**
  - Line 32: Added `dds_ip: Optional[str]` field to `HostConfig` (defaults to None)

- **core/models.py**
  - Line 14: Added `dds_ip: Optional[str] = None` to `Host` dataclass
  - Lines 16-19: Added `effective_dds_ip` property that returns `dds_ip` if set, otherwise falls back to `ip`

- **commands/stage.py**
  - Line 167: Use `dds_manager.effective_dds_ip` for superclient rendering
  - Line 186: Use `dds_manager.effective_dds_ip` for Dockerfile rendering
  - Lines 265-267: Find DDS manager for compose rendering
  - Line 273: Pass `dds_manager` to `render_compose()`

- **core/renderer.py**
  - Lines 71-72: In `render_compose()`, use `dds_manager.effective_dds_ip` if available, otherwise fall back to `cfg.discovery_server`

- **tests/unit/test_validation.py**
  - Lines 76-95: Added tests for `dds_ip` field validation

**Configuration:**
```yaml
hosts:
  - name: orin
    ip: nx128.local          # SSH/Docker communication
    user: cdr
    arch: arm64
    manager: true
    dds_ip: 10.0.0.100       # Separate DDS network (optional)
```

**Impact:** Enables separate networks for SSH/Docker traffic and DDS communication, useful for multi-homed systems or dedicated DDS networks.

---

### Improved Docker Error Handling
Docker command failures now show clean error messages instead of full Python tracebacks.

**What changed:**
- **cli.py**
  - Line 18: Added `from python_on_whales.exceptions import DockerException`
  - Lines 152-168: Added `DockerException` handler that:
    - Shows clean error message with simplified command (removes full binary paths)
    - Shows exit code
    - Points user to check output above for details
    - Avoids Python traceback clutter

- **commands/deploy.py**
  - Line 5: Added `from colorama import Fore`
  - Line 10: Added `from python_on_whales.exceptions import DockerException`
  - Lines 39-43: Wrapped `pull_image_on_host()` in try/except to add context before re-raising

- **commands/stage.py**
  - Line 8: Added `from colorama import Fore`
  - Line 13: Added `from python_on_whales.exceptions import DockerException`
  - Lines 261-265: Wrapped `pull_image_on_host()` in try/except to add context before re-raising

**Example output (before):**
```
[ERROR] Unhandled exception:
Traceback (most recent call last):
  File "/Users/cdr/Work/robocore-cli/cli.py", line 119, in main
    ...
  File "/opt/homebrew/lib/python3.13/site-packages/python_on_whales/utils.py", line 220, in run
    raise DockerException(...)
python_on_whales.exceptions.DockerException: The command executed was `/usr/local/bin/docker --host tcp://leremix-pi.local:2375 image pull docker.io/dragomirxyz/leremix_base:humble-latest`.
It returned with code 1
```

**Example output (after):**
```
[stage] Failed to pull base image on host 'pi' (leremix-pi.local)
[ERROR] Docker command failed

Command: docker --host tcp://leremix-pi.local:2375 image pull docker.io/dragomirxyz/leremix_base:humble-latest
Exit code: 1

Tip: Check the output above for details
```

**Impact:** Docker errors are now much more readable and user-friendly while still showing the actual error output from Docker.

---

## 2025-10-07 (Evening Session)

### Fixed Build Path Resolution
Fixed sync manager to use correct build directory paths.

**What changed:**
- **core/sync.py**
  - Line 17: Changed `self.build = os.path.abspath(cfg.build_dir)` to `self.build = os.path.join(cfg.root, cfg.build_dir)`
  - Line 23: Changed hardcoded `"build"` to `self.build` for proper path resolution

**Impact:** Sync now correctly finds build artifacts in `.robocore/build/` instead of looking in wrong paths.

---

### Added Component Image Pulling in Deploy
Deploy command now pulls component images on remote hosts before launching containers.

**What changed:**
- **commands/deploy.py**
  - Lines 31-37: Added loop to pull component images for each host before compose up
  - Filters components by `runs_on` to only pull relevant images per host

**Impact:** Remote hosts always use the latest staged images with all dependencies.

---

### Fixed Common Packages Sourcing in Docker Compose
Added conditional sourcing of common packages workspace in container startup commands.

**What changed:**
- **core/renderer.py**
  - Line 79: Added `has_common_packages = bool(cfg.common_packages)` to compose template context

- **templates/docker-compose.j2**
  - Lines 53-56, 71-74: Added conditional sourcing of `/ros_ws_common/install/setup.bash` if common packages exist
  - Only sources when `has_common_packages` is true

**Impact:** Components can now find packages from `common_packages` at runtime (e.g., `leremix_description`).

---

### Added Clean Command
New command to remove build artifacts and workspaces on local and remote machines.

**What changed:**
- **commands/clean.py** (NEW FILE)
  - `clean_main()` function to remove `.robocore` directories
  - Local cleanup: removes `.robocore/` in project root
  - Remote cleanup: stops containers, removes `mount_root` and `.robocore` directories on hosts
  - Uses SSH to execute cleanup commands on remote hosts

- **cli.py**
  - Line 15: Added `from commands.clean import clean_main`
  - Lines 88-98: Added `clean` subcommand with `-r/--remote` and `--local-only` flags

**Usage:**
```bash
# Clean local only
python3 cli.py clean

# Clean both local and remote
python3 cli.py clean -r

# Clean local only (explicit)
python3 cli.py clean --local-only
```

---

### Added Pip Package Support for Components
Components can now install Python packages via pip.

**What changed:**
- **core/models.py**
  - Line 87: Added `pip_packages: List[str] = field(default_factory=list)` to Component class
  - Line 127: Added `pip_packages= d.get('pip_packages', [])` to `from_dict()`

- **templates/Dockerfile.j2**
  - Lines 19-22: Added pip package installation step using `pip3 install --no-cache-dir {{ pip_packages | join(' ') }}`

- **core/renderer.py**
  - Line 30: Added `pip_packages` parameter to `render_dockerfile()`
  - Line 47: Pass `pip_packages=pip_packages` to template

- **commands/stage.py**
  - Line 181: Pass `pip_packages=comp.pip_packages or []` to renderer

**Configuration:**
```yaml
components:
  - name: leremix_control
    pip_packages:
      - st3215
      - numpy
```

---

### Improved Keyboard Interrupt Handling
Better handling of Ctrl-C to avoid full Python tracebacks.

**What changed:**
- **cli.py**
  - Lines 114-116: Added `KeyboardInterrupt` exception handler with clean exit message
  - Exit code 130 (standard for SIGINT)

- **commands/deploy.py**
  - Line 4: Added `import signal`
  - Lines 40-45: Wrapped `compose_up_remote()` in try/except to catch KeyboardInterrupt
  - Shows helpful message about how to stop containers

**Impact:** Ctrl-C now shows clean message instead of full traceback:
```
[deploy:orin] Interrupted by user. Containers may still be running.
[deploy:orin] To stop: docker compose -f docker-compose.orin.yaml down
```

---

### Fixed Rosdep Installation in Base Image
Added rosdep dependency resolution for common packages.

**What changed:**
- **templates/Dockerfile.base.j2**
  - Lines 48-50: Ensure `src/` directory is created before copying packages
  - Lines 72-75: Added `rosdep update` and `rosdep install --from-paths src --ignore-src -r -y` before building
  - Lines 44-46, 48, 88, 93-95: Refactored to only build/source workspace if common packages exist

**Impact:** Common packages now have their ROS dependencies automatically installed (e.g., `xacro` for URDF processing).

---

## 2025-10-07 (Earlier Session)

### Common Packages Format Update
Updated `common_packages` to support both VCS-based and local source-based packages.

**What changed:**
- **core/models.py**
  - Added `CommonPackage` class with support for both `repositories` and `source` fields
  - Updated `RepositorySpec` to auto-derive folder name from URL if not specified
  - `folder` field is now optional in repository specs

- **core/config.py**
  - Updated to parse `common_packages` using `CommonPackage.from_dict()` instead of `Component.from_dict()`
  - Imported `CommonPackage` model

- **templates/Dockerfile.base.j2**
  - Updated to handle both VCS-based packages (using vcstool with repos.yaml)
  - Added support for local source packages (direct COPY commands)
  - Split package handling based on `is_vcs_based` and `is_local_based` properties

- **README.md**
  - Updated common_packages documentation with new format examples
  - Documented both VCS-based and local source-based syntax

**New format:**
```yaml
common_packages:
  # VCS-based package (one or more repositories)
  - name: my_msgs
    repositories:
      - url: https://github.com/you/my_msgs.git
        version: main
        # folder is optional - auto-derived from URL

  # Local source package (one or more folders)
  - name: leremix_control
    source:
      - ros_ws/leremix_control
      - ros_ws/leremix_control_plugin
```

---

### Configurable Workspace Directory
Made the workspace directory name configurable via `config.yaml`.

**What changed:**
- **core/config.py**
  - Added `workspace_dir` config option (default: `"ros_ws"`)
  - Passed `workspace_dir` to `Component.from_dict()` during initialization

- **core/models.py**
  - Added `workspace_dir` field to `Component` dataclass
  - Updated `from_dict()` to accept and use `workspace_dir` parameter
  - Updated `comp_src`, `repos_file`, and `get_source_paths()` to use `self.workspace_dir` instead of hardcoded `"ros_ws"`

- **commands/stage.py**
  - Line 78: Updated legacy path construction to use `cfg.workspace_dir`
  - Line 149: Updated relative path construction to use `cfg.workspace_dir`

- **commands/build.py**
  - Line 42: Updated source directory path to use `cfg.workspace_dir`
  - Line 54: Updated workspace root path to use `cfg.workspace_dir`

- **commands/shell.py**
  - Line 64: Updated ros_ws_src path to use `cfg.workspace_dir`
  - Line 68: Updated ros_ws_install path to use `cfg.workspace_dir`

- **core/sync.py**
  - Added `workspace_dir` to `SyncManager.__init__()`
  - Lines 23, 29, 31: Updated rsync paths to use `self.workspace_dir`

- **commands/prepare_base.py**
  - Line 33: Added `workspace_dir=cfg.workspace_dir` parameter to `renderer.render_base()` call

- **core/renderer.py**
  - Line 81: Updated `render_base()` signature to accept `workspace_dir` parameter (default: "ros_ws")
  - Line 91: Pass `workspace_dir` to template rendering

**Configuration:**
```yaml
# Optional - defaults to "ros_ws" if not specified
workspace_dir: ros_ws
```
