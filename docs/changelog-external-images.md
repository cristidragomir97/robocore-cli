# Changelog: External Images and Custom Dockerfiles

## Summary

Added support for three component build strategies in forge:
1. **Forge-managed builds** (existing) - automated Dockerfile generation
2. **External pre-built images** (new) - use community images without building
3. **Custom Dockerfile builds** (new) - full control for complex builds

## Changes

### 1. Core Models (`core/models.py`)

Added two new fields to the `Component` dataclass:
- `build: Optional[str]` - Path to directory containing custom Dockerfile
- `image: Optional[str]` - External pre-built image reference

Updated `image_tag()` method to return the external image when specified.

### 2. Configuration Validation (`core/validation.py`)

Added fields to `ComponentConfig`:
- `build: Optional[str]`
- `image: Optional[str]`

Added validation logic:
- Ensures mutual exclusivity between `source`, `sources`, `folder`, `build`, and `image`
- Prevents mixing `image`/`build` with forge build options (`repositories`, `apt_packages`, `pip_packages`)
- Validates that custom Dockerfiles exist at specified paths

### 3. Stage Command (`commands/stage.py`)

Modified `build_component_image()` to handle three build modes:

**External Image Mode** (`if comp.image:`):
- Skips build entirely
- Returns component metadata with image reference
- Image is used as-is during deploy

**Custom Dockerfile Mode** (`if comp.build:`):
- Builds from user's Dockerfile in specified directory
- Uses the build directory as context
- Builds with `docker buildx` for target architecture
- Pushes to registry

**Forge-Managed Mode** (default):
- Existing behavior
- Generates Dockerfile with dependencies
- Builds in `.forge/workspaces/{component}/`

### 4. Documentation

**Created**:
- `docs/external-images.md` - Comprehensive guide with examples and migration paths
- `docs/example-mixed-components.yaml` - Complete example showing all three component types
- `docs/changelog-external-images.md` - This file

**Updated**:
- `README.md` - Components section now explains all three build types
- `README.md` - Stage command documentation updated with new flags

## Usage Examples

### External Image

```yaml
components:
  - name: rosboard
    image: thehale/rosboard:latest
    runs_on: workstation
    ports:
      - "8888:8888"
```

### Custom Dockerfile

```yaml
components:
  - name: camera_gpu
    build: docker/camera
    runs_on: orin
    devices:
      - /dev/video0:/dev/video0
```

Directory structure:
```
project_root/
  docker/
    camera/
      Dockerfile
      camera_driver/
```

### Forge-Managed (Existing)

```yaml
components:
  - name: motion
    source: ros/src/motion_control
    repositories:
      - url: https://github.com/ros-controls/ros2_control.git
        version: humble
    apt_packages:
      - ros-humble-controller-manager
    runs_on: robot
```

## Benefits

1. **Flexibility**: Choose the right build strategy for each component
2. **Community Integration**: Easy to use existing Docker images from the ecosystem
3. **Advanced Use Cases**: Custom Dockerfiles for GPU drivers, complex builds, etc.
4. **Backward Compatible**: Existing configurations work unchanged
5. **Validation**: Clear error messages prevent misconfiguration

## Migration Path

### From External to Custom Dockerfile

If you need to customize a community image:

1. Create `docker/{component}/Dockerfile` starting with the external image
2. Change `image: osrf/ros:humble` to `build: docker/{component}`
3. Add your customizations to the Dockerfile

### From Forge-Managed to Custom Dockerfile

If you need more control:

1. Generate Dockerfile during a normal stage run
2. Copy generated Dockerfile from `.forge/workspaces/{component}/`
3. Move to `docker/{component}/` and customize
4. Change component config from `source:` to `build: docker/{component}`

## Testing

Syntax validation:
```bash
python3 -m py_compile commands/stage.py core/models.py core/validation.py
```

Example project test:
```bash
# Test with example config
python3 cli.py -p examples/mixed stage
```

## Files Modified

- `core/models.py` - Added build/image fields
- `core/validation.py` - Added validation for new fields
- `commands/stage.py` - Implemented three-mode build logic
- `README.md` - Updated component documentation
- `docs/external-images.md` - New comprehensive guide
- `docs/example-mixed-components.yaml` - New example config
- `docs/changelog-external-images.md` - This changelog

## Backward Compatibility

✅ **Fully backward compatible**

Existing configurations using `source`, `sources`, `folder`, or `repositories` work unchanged. The new fields are optional and only activate when explicitly specified.

## Future Enhancements

Possible future improvements:
1. Support for Docker Compose extends
2. Multi-stage Dockerfile templates
3. Automatic Dockerfile generation from running containers
4. Build caching for custom Dockerfiles (similar to base image caching)
5. Pre-build hooks for custom builds
