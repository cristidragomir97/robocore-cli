# Workspace Management Redesign Proposal

## Overview

This proposal outlines a redesign of forge's workspace management to improve flexibility, support multiple source packages per component, and provide better isolation between managed workspaces and user source code.

## Current Issues

1. **Rigid workspace structure**: Components must follow `{folder}/ros_ws/src` structure
2. **Single package limitation**: One component = one folder/package
3. **Source code mixing**: User source code lives in forge managed directories
4. **Inflexible paths**: Cannot reference existing ROS packages in arbitrary locations

## Proposed Solution

### 1. Managed Workspace Directory

Introduce a `.forge/` directory within each project to house all managed workspaces and build artifacts:

```
my_robot_project/
├── config.yaml
├── .forge/
│   ├── workspaces/
│   │   ├── perception/         # Component workspace
│   │   │   ├── src/           # Symlinks to source packages
│   │   │   ├── build/
│   │   │   └── install/
│   │   └── navigation/
│   │       ├── src/
│   │       ├── build/
│   │       └── install/
│   ├── repos/                 # VCS dependencies
│   └── cache/                # Build caches
├── my_packages/              # User's source code
│   ├── perception_pkg/
│   ├── camera_driver/
│   └── common_msgs/
└── external_packages/        # More user source code
    └── navigation_stack/
```

### 2. Enhanced Component Configuration

Replace the single `folder` field with multiple source paths and rename for clarity:

#### Current Configuration
```yaml
components:
  - name: perception
    folder: components/perception  # Single rigid path
```

#### Proposed Configuration
```yaml
components:
  - name: perception
    sources:                     # Multiple source paths
      - path: my_packages/perception_pkg
      - path: my_packages/camera_driver
      - path: external_packages/shared_msgs
    # OR single path for backwards compatibility
    source: my_packages/perception_pkg
```

### 3. Component Model Changes

#### New Component Fields

```python
@dataclass
class Component:
    name: str
    sources: List[str] = field(default_factory=list)  # Multiple source paths
    source: Optional[str] = None                      # Single source (backward compat)

    # Remove these fields (managed automatically):
    # folder: str  # Replaced by managed workspace

    # Keep existing fields:
    entrypoint: str = ""
    launch_args: str = ""
    devices: List[str] = field(default_factory=list)
    ports: List[str] = field(default_factory=list)
    preinstall: List[str] = field(default_factory=list)
    postinstall: List[str] = field(default_factory=list)
    runs_on: Optional[str] = None
    repositories: List[RepositorySpec] = field(default_factory=list)
    apt_packages: List[str] = field(default_factory=list)
    simulate: bool = False
    image: Optional[str] = None

    @property
    def managed_workspace(self) -> str:
        """Path to the managed workspace in .forge/workspaces/{name}"""
        return f".forge/workspaces/{self.name}"

    @property
    def workspace_src(self) -> str:
        """Path to src directory in managed workspace"""
        return os.path.join(self.managed_workspace, "src")

    def get_source_paths(self) -> List[str]:
        """Get all source paths for this component"""
        if self.sources:
            return self.sources
        elif self.source:
            return [self.source]
        return []
```

## Implementation Changes

### 1. Core Models (`core/models.py`)

```python
# Replace current properties
@property
def comp_src(self):
    # OLD: return os.path.join(self.folder, "ros_ws", "src")
    return self.workspace_src

@property
def workspace_path(self):
    """Full path to managed workspace"""
    return self.managed_workspace
```

### 2. Staging Process (`commands/stage.py`)

```python
def analyze_component(comp: Component, cfg) -> Dict[str, Any]:
    """Analyze component and prepare workspace"""

    # Create managed workspace
    workspace_dir = os.path.join(cfg.root, comp.managed_workspace)
    src_dir = os.path.join(workspace_dir, "src")
    os.makedirs(src_dir, exist_ok=True)

    # Link source packages
    source_paths = comp.get_source_paths()
    has_local_sources = False

    for source_path in source_paths:
        abs_source = os.path.join(cfg.root, source_path)
        if os.path.exists(abs_source):
            # Create symlink in managed workspace
            package_name = os.path.basename(abs_source)
            link_target = os.path.join(src_dir, package_name)

            if os.path.exists(link_target):
                os.unlink(link_target)
            os.symlink(abs_source, link_target)
            has_local_sources = True
            print(f"  - linked: {source_path} -> {comp.name}/src/{package_name}")

    return {
        "has_local": has_local_sources,
        "workspace_path": workspace_dir,
        # ... other features
    }
```

### 3. Build Process (`commands/build.py`)

```python
def build_main(project_root: str, component_name=None):
    """Build components using managed workspaces"""

    for comp in comps:
        workspace_dir = os.path.join(cfg.root, comp.managed_workspace)
        src_dir = os.path.join(workspace_dir, "src")

        if not os.path.exists(src_dir) or not os.listdir(src_dir):
            print(f"[build] Skipping '{comp.name}' (no sources linked)")
            continue

        # Build in the managed workspace
        print(f"[build] Building '{comp.name}' in {workspace_dir}")
        # ... rest of build logic
```

### 4. Configuration Loading

Support both old and new formats with automatic migration warnings:

```python
@classmethod
def from_dict(cls, d, is_common=False):
    # Handle new multi-source format
    sources = d.get('sources', [])
    source = d.get('source')

    # Backward compatibility for 'folder' field
    if 'folder' in d and not sources and not source:
        print(f"WARNING: Component '{d['name']}' uses deprecated 'folder' field. "
              f"Consider migrating to 'source' or 'sources' fields.")
        source = d['folder']

    return cls(
        name=d['name'],
        sources=sources,
        source=source,
        # ... rest of fields
    )
```

## Usage Examples

### Example 1: Multiple Packages Per Component

```yaml
components:
  - name: perception_system
    sources:
      - path: packages/camera_driver
      - path: packages/lidar_driver
      - path: packages/perception_fusion
      - path: shared/sensor_msgs
    runs_on: jetson_xavier
    entrypoint: ros2 launch perception_fusion main.launch.py
    devices:
      - "/dev/video0:/dev/video0"
```

### Example 2: Single Package Component

```yaml
components:
  - name: navigation
    source: packages/nav_stack
    runs_on: main_computer
    entrypoint: ros2 launch nav_stack navigation.launch.py
```

### Example 3: External Dependencies Only

```yaml
components:
  - name: micro_ros_agent
    repositories:
      - url: https://github.com/micro-ROS/micro_ros_setup
        version: humble
        folder: micro_ros_setup
    runs_on: gateway
    entrypoint: ros2 run micro_ros_agent micro_ros_agent
```

## Migration Path

### Phase 1: Backward Compatibility
- Support both `folder` and new `source`/`sources` fields
- Issue deprecation warnings for `folder` usage
- Generate managed workspaces automatically

### Phase 2: Enhanced Features
- Add workspace caching and incremental builds
- Support workspace overlays for common packages
- Add validation for source path existence

### Phase 3: Full Migration
- Remove `folder` field support
- Optimize workspace management performance
- Add advanced workspace features

## Benefits

1. **Flexibility**: Reference packages from anywhere in your project
2. **Modularity**: Combine multiple packages into logical components
3. **Isolation**: Clean separation between managed and user code
4. **Maintainability**: No more confusion about where code should live
5. **Scalability**: Support complex multi-package robot systems
6. **Developer Experience**: Work with existing ROS workspace structures

## Files Requiring Changes

### Core Files
- `core/models.py`: Update Component class and properties
- `core/config.py`: Add `.forge` directory handling

### Command Files
- `commands/stage.py`: Implement workspace linking logic
- `commands/build.py`: Update to use managed workspaces
- `commands/init.py`: Update scaffold generation
- `core/sync.py`: Update sync paths to managed workspaces

### Templates
- `templates/Dockerfile.j2`: Update COPY commands for new paths
- `templates/docker-compose.j2`: Update volume mounts

This redesign maintains backward compatibility while providing the flexibility to reference existing ROS packages and support multiple packages per component, addressing the core limitations of the current system.