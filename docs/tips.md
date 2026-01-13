# Tips & Troubleshooting

Quality of life tips and solutions for common issues.

## Docker Host Setup

### Enabling TCP Socket

Forge requires Docker hosts to listen on a TCP socket for remote operations.

#### Step-by-Step Setup

1. Create `/etc/docker/daemon.json`:
   ```json
   {
     "hosts": ["tcp://0.0.0.0:2375", "unix:///var/run/docker.sock"]
   }
   ```

2. Create systemd override at `/etc/systemd/system/docker.service.d/override.conf`:
   ```ini
   [Service]
   ExecStart=
   ExecStart=/usr/bin/dockerd
   ```

3. Reload and restart Docker:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl restart docker
   ```

4. Verify it's working:
   ```bash
   curl http://localhost:2375/version
   ```

#### Security Considerations

The default configuration exposes Docker without authentication. This is fine for:
- Home/lab networks
- Development environments
- Networks you control

For production or untrusted networks, consider:
- SSH tunneling: `ssh -L 2375:localhost:2375 user@host`
- TLS authentication (see Docker docs)
- VPN between hosts

---

## Build Optimization

### Faster ARM64 Builds

Building ARM64 images on x86_64 is slow due to QEMU emulation. Solutions:

#### Use Apple Silicon Mac

If available, Macs with M1/M2/M3 chips build ARM64 natively.

#### Build on Device (Recommended for x86_64 hosts)

Set `build_on_device: true` to build everything natively on the target device:

```yaml
hosts:
  - name: jetson
    ip: jetson.local
    user: nvidia
    arch: arm64
    build_on_device: true  # All builds happen on the Jetson
```

This affects all build steps:
- **prep**: Base image is built on the device
- **stage**: Component images are built on the device
- **build**: ROS workspace is compiled on the device

The build context is synced to the device via rsync, built there, and pushed to the registry. This is typically 10x faster than QEMU emulation.

#### Use Build Cache

Forge caches base images. Avoid `--force-base` unless necessary:

```bash
# Only rebuilds if config changed
forge my_robot prep

# Forces full rebuild (slow)
forge my_robot prep --force-base
```

### Apt Mirror Configuration

Speed up package downloads by using a local mirror:

```yaml
apt_mirror: http://mirror.local/ubuntu
ros_apt_mirror: http://mirror.local/ros2/ubuntu
```

### Enable Apt Caching

For repeated builds, enable apt caching:

```yaml
enable_apt_caching: true
```

---

## Debugging

### Container Won't Start

1. Check logs:
   ```bash
   docker logs <container_name>
   ```

2. Verify image exists:
   ```bash
   docker images | grep <image_name>
   ```

3. Try running interactively:
   ```bash
   docker run -it --rm <image_name> bash
   ```

### Nodes Can't Discover Each Other

1. Verify middleware router is running:
   ```bash
   # For FastDDS
   docker ps | grep dds_server

   # For Zenoh
   docker ps | grep zenoh_router
   ```

2. Check environment variables:
   ```bash
   docker exec <container> env | grep -E "RMW|ROS|ZENOH|FASTRTPS"
   ```

3. Test connectivity:
   ```bash
   # From container
   ros2 topic list
   ros2 node list
   ```

### Build Fails

1. Check for missing dependencies:
   ```bash
   # Run rosdep check
   rosdep check --from-paths src --ignore-src
   ```

2. Verify source paths exist:
   ```bash
   ls -la <source_path>
   ```

3. Check for package.xml:
   ```bash
   find src -name package.xml
   ```

---

## Common Patterns

### Reusable Component Templates

Create a base component config and extend it:

```yaml
# Define common settings
x-common-component: &common
  nvidia: true
  shm_size: 2g
  environment:
    ROS_LOG_DIR: /tmp/ros_logs

components:
  - name: perception
    <<: *common
    source: components/perception
    runs_on: jetson

  - name: planning
    <<: *common
    source: components/planning
    runs_on: jetson
```

### Development vs Production Configs

Maintain separate config files:

```
project/
├── config.yaml           # Production config
├── config.dev.yaml       # Development config
└── config.local.yaml     # Local-only testing
```

Use with:
```bash
forge my_robot stage --config config.dev.yaml
```

### Quick Iteration Loop

For fast development cycles:

```bash
# After code changes
forge my_robot build -c my_component && forge my_robot launch --host robot
```

Or create a script:

```bash
#!/bin/bash
# dev.sh
forge . build -c $1 && forge . launch
```

---

## Hardware Access

### USB Devices

Map specific devices:

```yaml
components:
  - name: lidar
    devices:
      - "/dev/ttyUSB0:/dev/ttyUSB0"
```

Or map all USB devices (less secure):

```yaml
components:
  - name: sensors
    devices:
      - "/dev:/dev"
    privileged: true
```

### Persistent Device Names

USB devices can change paths. Use udev rules for stable names:

```bash
# /etc/udev/rules.d/99-robot.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", SYMLINK+="lidar"
```

Then use:
```yaml
devices:
  - "/dev/lidar:/dev/lidar"
```

### GPIO Access

For Raspberry Pi GPIO:

```yaml
components:
  - name: gpio_controller
    devices:
      - "/dev/gpiomem:/dev/gpiomem"
    privileged: true
```

### Camera Access

```yaml
components:
  - name: camera
    devices:
      - "/dev/video0:/dev/video0"
```

---

## Networking Tips

### Finding the Right IP

If hosts have multiple network interfaces, specify `dds_ip`:

```yaml
hosts:
  - name: robot
    ip: robot.local        # For SSH
    dds_ip: 192.168.1.100  # For ROS communication
```

### Dealing with Firewalls

Ensure these ports are open:

| Middleware | Ports |
|------------|-------|
| FastDDS | 11811 (TCP/UDP), 7400-7500 (UDP) |
| Zenoh | 7447 (TCP) |

### Testing Connectivity

```bash
# From dev machine to robot
ping <robot_ip>
nc -zv <robot_ip> 2375   # Docker
nc -zv <robot_ip> 11811  # FastDDS
nc -zv <robot_ip> 7447   # Zenoh
```

---

## Cleanup

### Remove Old Images

```bash
# On all hosts
docker image prune -a
```

### Clean Build Artifacts

```bash
rm -rf .forge/build/*
```

### Reset Everything

```bash
rm -rf .forge/
forge my_robot prep --force-base
forge my_robot stage
forge my_robot build
```
