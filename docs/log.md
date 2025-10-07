# Changelog

## 2025-10-07

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
