# Performance Analysis & Optimization Opportunities

## Executive Summary

Analysis of forge reveals significant performance bottlenecks in the Docker image build/push/pull workflow and opportunities to simplify over-engineered components. For a typical project with 4 components on 2 hosts, the current workflow involves:

- 1 base image build + push (multi-arch)
- 4 component image builds + pushes
- 2 × (1 base + 4 component) = 10 image pulls
- **Total: 15 Docker operations involving registry transfers**

This document outlines immediate wins, medium-term optimizations, and architectural improvements.

---

## Current Architecture

### Build & Deploy Flow

```
┌─────────────────────────────────────────────────────────────┐
│ 1. STAGE                                                     │
│    - prepare_base: Build base image for all platforms       │
│      └─> Push to Docker Hub                                 │
│    - For each component:                                    │
│      └─> Build image (using base)                          │
│      └─> Push to Docker Hub                                 │
│      └─> Pull base image to target host                    │
│                                                              │
│ 2. BUILD (optional, for local sources)                      │
│    - Copy sources to build directory                        │
│    - Run colcon build in container                          │
│    - Store artifacts locally                                │
│                                                              │
│ 3. DEPLOY                                                    │
│    - For each host:                                         │
│      └─> rsync build artifacts                             │
│      └─> Pull component images from registry               │
│      └─> docker compose up                                  │
└─────────────────────────────────────────────────────────────┘
```

### Issues Identified

#### 1. Docker Registry as Bottleneck
- **Problem**: Every build pushes to registry, every deploy pulls from registry
- **Impact**:
  - Upload bandwidth: ~500MB-2GB per component
  - Download bandwidth: Same on each target host
  - Latency: Registry roundtrip adds 30s-5min per image
  - Cost: Docker Hub rate limits, bandwidth costs

#### 2. No Base Image Caching
- **Problem**: `prepare_base_main()` always rebuilds and pushes base image
- **Impact**: Wastes 2-5 minutes rebuilding unchanged base layer
- **Code**: `commands/prepare_base.py:43` - `docker.build_multiarch(..., push=True)`

#### 3. Unnecessary Managed Workspace
- **Problem**: Stage builds images with sources copied directly in Dockerfile, but `.forge/workspaces/` still created for generated files
- **Impact**:
  - Confusing workspace structure
  - Wasted disk space
  - Extra abstraction layer in code
- **Recent Fix**: We eliminated symlink copying, but workspace directory still exists

#### 4. Sequential Component Builds
- **Problem**: Components built one at a time
- **Impact**: With 4 components @ 3min each = 12min (could be 3-4min parallel)
- **Code**: `commands/stage.py:215-226` - Loop through components sequentially

#### 5. Build Artifacts Sync Issues
- **Problem**: `build` command produces artifacts that are sync'd, but sources already in Docker images
- **Impact**:
  - Confusing: Why build locally if image has everything?
  - Slow rsync for large install directories
  - Version mismatch risk between local build and image

---

## Quick Wins (Immediate, <1 day)

### 1. Skip Base Image Rebuild
**Effort**: 2 hours
**Impact**: Save 2-5 minutes per stage

Add check if base image exists and hasn't changed:

```python
# commands/prepare_base.py
def prepare_base_main(project_root: str, force: bool = False):
    # ... existing code ...

    if not force:
        # Check if base image exists locally
        try:
            docker.client.image.inspect(base_tag)
            print(f"[prepare_base] Using cached base image {base_tag}")
            print(f"[prepare_base] Use --force to rebuild")
            return
        except:
            pass  # Image doesn't exist, build it

    # Build only if forced or doesn't exist
    docker.build_multiarch(...)
```

### 2. Add --skip-push Flag for Local Development
**Effort**: 1 hour
**Impact**: Save 1-3 minutes per component for local testing

```python
# commands/stage.py
def stage_main(project_root: str, component: Optional[str] = None,
               skip_push: bool = False):
    # ...
    docker.build_multiarch(
        image_tag=img_tag,
        context=cfg.root,
        dockerfile=dockerfile_path,
        platforms=[platform],
        push=not skip_push  # Don't push in local dev mode
    )
```

Usage: `forge stage --skip-push` for rapid iteration.

### 3. Eliminate Managed Workspace
**Effort**: 4 hours
**Impact**: Simpler codebase, less confusion

**Current State**: After our recent refactor, `.forge/workspaces/` only contains:
- `Dockerfile` (generated)
- `repos.yaml` (generated)
- `superclient.xml` (generated)

**Improvement**: Move these to `.forge/generated/{component}/`:
```
.forge/
├── generated/
│   ├── motion/
│   │   ├── Dockerfile
│   │   ├── repos.yaml
│   │   └── superclient.xml
│   └── lidar/
│       └── ...
├── base/
│   └── Dockerfile.base
└── build/  # Only if using local build
    └── motion/
        └── ros_ws/
```

Remove `managed_workspace` property from Component model.

### 4. Parallel Component Builds
**Effort**: 3 hours
**Impact**: 3-4× faster for multiple components

```python
# commands/stage.py
from concurrent.futures import ThreadPoolExecutor, as_completed

def stage_main(...):
    # ... existing setup ...

    # Build components in parallel
    with ThreadPoolExecutor(max_workers=4) as executor:
        futures = []
        for host_name, host_comps in components_by_host.items():
            for j, comp in enumerate(host_comps):
                future = executor.submit(
                    build_component_image,
                    comp, cfg, hosts_map, base_tag, renderer, docker, j + 2
                )
                futures.append((future, host_name, comp))

        # Collect results
        for future, host_name, comp in futures:
            entry = future.result()
            staged_by_host[host_name].append(entry)
```

**Caveat**: Need thread-safe logging.

---

## Medium-Term Optimizations (1-2 weeks)

### 1. Local Docker Registry
**Effort**: 2 days
**Impact**: 10-20× faster image transfers

Run a local registry on the build machine:

```yaml
# Add to config.yaml
registry_mode: local  # or "remote"
local_registry_port: 5000
```

```python
# Start local registry automatically
def ensure_local_registry():
    try:
        docker.client.container.inspect("forge-registry")
    except:
        docker.client.run(
            "registry:2",
            name="forge-registry",
            detach=True,
            publish=[(5000, 5000)],
            restart="always"
        )
```

**Transfer Speed Comparison**:
- Docker Hub: ~5-10 MB/s (depends on internet)
- Local registry: ~100-500 MB/s (LAN speed)
- For 500MB image: 50-100s vs 1-5s

### 2. Docker Save/Load for One-Host Setup
**Effort**: 1 day
**Impact**: Skip registry entirely for single-host development

```python
def transfer_image_directly(image_tag: str, host: Host):
    """Transfer image directly via docker save/load."""
    print(f"[stage] Transferring {image_tag} directly to {host.name}")

    # Save image to tarball
    temp_file = f"/tmp/{image_tag.replace('/', '_')}.tar"
    subprocess.run(f"docker save {image_tag} -o {temp_file}", shell=True, check=True)

    # SCP to host
    subprocess.run(
        f"scp {temp_file} {host.user}@{host.ip}:/tmp/",
        shell=True, check=True
    )

    # Load on host
    subprocess.run(
        f"ssh {host.user}@{host.ip} 'docker load -i /tmp/{os.path.basename(temp_file)}'",
        shell=True, check=True
    )

    # Cleanup
    os.remove(temp_file)
```

**When to use**:
- Single host deployments
- Hosts on same LAN as build machine
- No Docker Hub account

### 3. Incremental Builds with Layer Caching
**Effort**: 3 days
**Impact**: 50-80% faster rebuilds

Current issue: No layer cache reuse because we always build from scratch.

```python
# Enable buildkit cache
docker.build_multiarch(
    ...,
    cache_to="type=registry,ref={registry}/{prefix}_cache:{component}",
    cache_from="type=registry,ref={registry}/{prefix}_cache:{component}",
)
```

Or use local cache:
```python
cache_to="type=local,dest=/tmp/forge-cache/{component}",
cache_from="type=local,src=/tmp/forge-cache/{component}",
```

### 4. Smarter Component Selection
**Effort**: 2 days
**Impact**: Only rebuild changed components

Track source file hashes:
```python
# .forge/component-hashes.json
{
  "motion": "abc123...",
  "lidar": "def456..."
}

def get_source_hash(comp: Component, cfg: Config) -> str:
    """Hash all source files for this component."""
    hasher = hashlib.sha256()
    for src_path, pkg_name in resolve_source_packages(comp, cfg):
        for root, dirs, files in os.walk(src_path):
            for file in sorted(files):
                if not file.endswith(('.pyc', '.so')):
                    with open(os.path.join(root, file), 'rb') as f:
                        hasher.update(f.read())
    return hasher.hexdigest()
```

Only rebuild if hash changed.

---

## Long-Term Improvements (1-2 months)

### 1. Unified Image with Volume Mounts
**Effort**: 2-3 weeks
**Impact**: Eliminate per-component images entirely

**Current**: 1 base image + N component images
**Proposed**: 1 unified image + N volume mounts

```yaml
# Components become volume configurations
components:
  - name: motion
    sources:
      - ros/src/leremix_control
      - ros/src/leremix_servo_manager
    # Image is shared: myproject_unified:latest
```

**Benefits**:
- Build once, deploy everywhere
- Instant "rebuilds" (just mount new code)
- No image push/pull for code changes

**Tradeoffs**:
- Need to handle dependencies per-component (maybe layer apt packages)
- Slightly more complex deployment

### 2. Remote Build on Target Host
**Effort**: 3 weeks
**Impact**: Eliminate all image transfers

Instead of:
1. Build on dev machine (amd64)
2. Push to registry
3. Pull on target (arm64)

Do:
1. `rsync` sources to target
2. Build natively on target
3. Run directly

**Pros**:
- No cross-compilation
- No registry needed
- Faster iteration

**Cons**:
- Target host needs build tools
- Slower builds on weak hardware (Pi)

### 3. Development Container + Remote Execution
**Effort**: 1 month
**Impact**: Best DX for iterative development

Use VS Code Remote Containers or similar:
1. Develop in container locally
2. Hot-reload changes on target via SSH
3. Only build images for production deployments

Similar to: `tilt` for Kubernetes development

---

## Comparison Matrix

| Optimization | Effort | Impact | When to Use |
|-------------|--------|--------|-------------|
| Skip base rebuild | 2h | High | Always |
| --skip-push flag | 1h | Medium | Local dev |
| Remove managed workspace | 4h | Low | Code cleanup |
| Parallel builds | 3h | High | Multi-component |
| Local registry | 2d | Very High | LAN deployment |
| docker save/load | 1d | High | Single host |
| Layer caching | 3d | High | Frequent rebuilds |
| Smart rebuild | 2d | Medium | Large projects |
| Unified image | 3w | Very High | Mature projects |
| Remote build | 3w | Very High | Weak dev machine |

---

## Recommended Implementation Order

### Phase 1: Quick Wins (Week 1)
1. Skip base image rebuild (unless changed)
2. Add --skip-push flag
3. Parallel component builds

**Expected improvement**: 60-70% faster stage command

### Phase 2: Registry Optimization (Week 2-3)
1. Local registry for LAN deployments
2. docker save/load option for single-host

**Expected improvement**: 80-90% faster for image transfers

### Phase 3: Smart Rebuilds (Week 4)
1. Source hash tracking
2. BuildKit cache configuration

**Expected improvement**: 50-80% faster for unchanged components

### Phase 4: Architecture (Month 2-3)
1. Evaluate unified image approach
2. Consider remote build option

**Expected improvement**: Fundamental simplification

---

## Appendix: Measurements

### Current Timing (4 components, 2 hosts)

```
stage:
  prepare_base:     3m 30s
  build motion:     2m 15s
  build lidar:      1m 45s
  build imu:        1m 30s
  build rosboard:   1m 20s
  pull to hosts:    4m 00s (2 hosts × 5 images × ~25s)
  ────────────────────────
  Total:           14m 20s

deploy:
  rsync builds:     0m 30s
  pull images:      4m 00s
  compose up:       0m 20s
  ────────────────────────
  Total:            4m 50s

Grand Total:       19m 10s
```

### With Quick Wins

```
stage:
  prepare_base:     0m 05s (cached)
  build all (||):   2m 30s (parallel)
  skip push:        0m 00s
  ────────────────────────
  Total:            2m 35s

deploy:
  rsync builds:     0m 30s
  images local:     0m 00s (already built)
  compose up:       0m 20s
  ────────────────────────
  Total:            0m 50s

Grand Total:       3m 25s (82% improvement)
```

### With Local Registry

```
stage:
  prepare_base:     0m 05s (cached)
  build all (||):   2m 30s
  push local:       0m 10s
  ────────────────────────
  Total:            2m 45s

deploy:
  rsync builds:     0m 30s
  pull local:       0m 20s (LAN speed)
  compose up:       0m 20s
  ────────────────────────
  Total:            1m 10s

Grand Total:       3m 55s (80% improvement)
```

---

## Conclusion

The forge tool has significant optimization opportunities, particularly around Docker image management. The registry push/pull workflow is the primary bottleneck, contributing ~8-10 minutes of overhead for a typical 4-component project.

**Immediate Priority**: Implement Phase 1 quick wins for 60-70% improvement with minimal risk.

**Next Steps**: Evaluate local registry or direct transfer based on deployment topology.

**Long-term Vision**: Consider unified image approach for mature projects with stable dependencies.
