# SBC Test Farm Setup Guide

This guide explains how to set up your Single Board Computer (SBC) farm for robocore-cli end-to-end testing.

## Prerequisites

### Hardware Requirements
- 2+ SBCs (Raspberry Pi 4, Jetson Nano, Orange Pi, etc.)
- Network connectivity between all SBCs
- One coordinator machine (can be your development machine)
- SD cards with sufficient storage (32GB+ recommended)

### Supported Architectures
- **ARM64**: Raspberry Pi 4, Jetson Nano, Orange Pi 5
- **AMD64**: Intel NUC, mini PCs
- **Mixed environments**: Ideal for cross-platform testing

## SBC Farm Configuration

### 1. Network Setup

```bash
# Example home network configuration
# Router: 192.168.1.1
# Coordinator: 192.168.1.100 (your dev machine)
# SBC Farm subnet: 192.168.1.101-120

# Static IP assignments (configure in router or per-device)
192.168.1.101 - rpi4-manager     (ARM64, Manager role)
192.168.1.102 - rpi4-worker1    (ARM64, Worker)
192.168.1.103 - rpi4-worker2    (ARM64, Worker)
192.168.1.104 - jetson-nano     (ARM64, GPU worker)
192.168.1.105 - nuc-mini        (AMD64, Worker)
```

### 2. Base Image Preparation

Create a base SD card image with common setup:

```bash
#!/bin/bash
# prepare_sbc_image.sh

# Download Ubuntu Server 22.04 LTS for ARM64
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz

# Flash to SD card (replace /dev/sdX with your SD card device)
xz -dc ubuntu-22.04.2-preinstalled-server-arm64+raspi.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Mount the image for customization
sudo mkdir -p /mnt/boot /mnt/root
sudo mount /dev/sdX1 /mnt/boot
sudo mount /dev/sdX2 /mnt/root

# Enable SSH and set up initial user
sudo touch /mnt/boot/ssh
echo "robocore-test ALL=(ALL) NOPASSWD:ALL" | sudo tee -a /mnt/root/etc/sudoers

# Copy setup script
sudo cp sbc_setup.sh /mnt/root/home/ubuntu/

# Unmount
sudo umount /mnt/boot /mnt/root
```

### 3. Automated SBC Setup

Create `/home/ubuntu/sbc_setup.sh` on each SBC:

```bash
#!/bin/bash
# sbc_setup.sh - Run this on each SBC after first boot

set -e

# Configuration variables
COORDINATOR_IP="192.168.1.100"
SBC_ROLE="${1:-worker}"  # manager, worker, gpu-worker
SBC_NAME="${2:-$(hostname)}"

echo "=== Setting up $SBC_NAME as $SBC_ROLE ==="

# Update system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y \
    curl wget git vim htop net-tools iputils-ping \
    openssh-server python3 python3-pip python3-venv \
    docker.io docker-compose-plugin \
    build-essential cmake

# Add user to docker group
sudo usermod -aG docker $USER

# Configure Docker for remote access (test farm only!)
sudo mkdir -p /etc/docker
cat << EOF | sudo tee /etc/docker/daemon.json
{
  "hosts": ["unix:///var/run/docker.sock", "tcp://0.0.0.0:2375"],
  "insecure-registries": ["$COORDINATOR_IP:5000", "192.168.1.0/24"]
}
EOF

# Create systemd override
sudo mkdir -p /etc/systemd/system/docker.service.d
cat << EOF | sudo tee /etc/systemd/system/docker.service.d/override.conf
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd
EOF

sudo systemctl daemon-reload
sudo systemctl enable docker
sudo systemctl start docker

# Install ROS 2 Humble
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rmw-fastrtps-cpp \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool

# Initialize rosdep
sudo rosdep init
rosdep update

# Source ROS in bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# Install robocore-cli from coordinator
pip3 install git+http://$COORDINATOR_IP:8000/robocore-cli.git

# Setup SSH key access
mkdir -p ~/.ssh
curl -f http://$COORDINATOR_IP:8000/ssh/test_farm.pub >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
chmod 700 ~/.ssh

# Role-specific setup
case $SBC_ROLE in
    "manager")
        echo "=== Setting up as Manager ==="

        # Start Docker registry
        docker run -d -p 5000:5000 --restart=always --name registry registry:2

        # Configure DDS Discovery Server
        sudo mkdir -p /opt/dds-router
        cat << EOF | sudo tee /opt/dds-router/config.yaml
version: v4.0
participants:
  - name: DiscoveryServer
    kind: local-discovery-server
    id: 0
    listening-addresses:
      - ip: $(hostname -I | awk '{print $1}')
        port: 11811
        transport: udp
EOF

        # Set manager environment
        echo 'export ROS_DISCOVERY_SERVER=$(hostname -I | awk "{print \$1}"):11811' >> ~/.bashrc
        echo 'export ROBOCORE_ROLE=manager' >> ~/.bashrc

        # Mark as manager
        touch /tmp/is-manager
        ;;

    "worker"|"gpu-worker")
        echo "=== Setting up as Worker ==="

        # Configure DDS client
        cat << EOF > ~/.fastdds_discovery_profile.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="client" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>CLIENT</discoveryProtocol>
          <discoveryServersList>
            <RemoteServer prefix="4D.49.47.55.45.4c.5f.42.41.52.52.4f">
              <metatrafficUnicastLocatorList>
                <locator>
                  <udpv4>
                    <address>192.168.1.101</address>
                    <port>11811</port>
                  </udpv4>
                </locator>
              </metatrafficUnicastLocatorList>
            </RemoteServer>
          </discoveryServersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF

        echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=~/.fastdds_discovery_profile.xml' >> ~/.bashrc
        echo 'export ROS_DISCOVERY_SERVER=192.168.1.101:11811' >> ~/.bashrc
        echo "export ROBOCORE_ROLE=$SBC_ROLE" >> ~/.bashrc

        # GPU-specific setup for Jetson devices
        if [ "$SBC_ROLE" = "gpu-worker" ]; then
            # Install NVIDIA Container Toolkit (if Jetson)
            if lscpu | grep -q "aarch64"; then
                # Add NVIDIA Docker repo and install toolkit
                distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
                curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
                curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
                    sudo tee /etc/apt/sources.list.d/nvidia-docker.list
                sudo apt update && sudo apt install -y nvidia-docker2
                sudo systemctl restart docker
            fi
            touch /tmp/is-gpu-worker
        fi

        touch /tmp/is-worker
        ;;
esac

# Install monitoring tools
pip3 install psutil prometheus-client

# Create test farm info
cat << EOF > ~/.robocore_farm_info
{
  "name": "$SBC_NAME",
  "role": "$SBC_ROLE",
  "ip": "$(hostname -I | awk '{print $1}')",
  "arch": "$(uname -m)",
  "setup_time": "$(date -Iseconds)"
}
EOF

echo "=== SBC setup complete for $SBC_NAME ($SBC_ROLE) ==="
echo "Reboot recommended to ensure all services start properly"
```

## Test Farm Coordinator Setup

On your development/coordinator machine:

```bash
#!/bin/bash
# setup_coordinator.sh

# Start local Docker registry
docker run -d -p 5000:5000 --restart=always --name registry registry:2

# Start simple HTTP server for script distribution
cd /path/to/robocore-cli
python3 -m http.server 8000 &

# Generate SSH keys for farm access
ssh-keygen -t rsa -b 2048 -f ~/.ssh/test_farm_key -N ""

# Copy public key to HTTP server
mkdir -p ssh
cp ~/.ssh/test_farm_key.pub ssh/test_farm.pub

# Create farm configuration
cat << EOF > tests/e2e/sbc_farm_config.yaml
farm_name: "home_sbc_farm"
coordinator_ip: "192.168.1.100"
ssh_key: "~/.ssh/test_farm_key"

nodes:
  - name: "rpi4-manager"
    ip: "192.168.1.101"
    arch: "arm64"
    role: "manager"
    specs:
      cpu: "4x Cortex-A72"
      memory: "8GB"

  - name: "rpi4-worker1"
    ip: "192.168.1.102"
    arch: "arm64"
    role: "worker"
    specs:
      cpu: "4x Cortex-A72"
      memory: "4GB"

  - name: "rpi4-worker2"
    ip: "192.168.1.103"
    arch: "arm64"
    role: "worker"

  - name: "jetson-nano"
    ip: "192.168.1.104"
    arch: "arm64"
    role: "gpu-worker"
    specs:
      gpu: "128-core Maxwell"

  - name: "nuc-mini"
    ip: "192.168.1.105"
    arch: "amd64"
    role: "worker"
    specs:
      cpu: "Intel i5"
      memory: "16GB"
EOF
```

## Running SBC Farm Tests

### 1. Basic Connectivity Test

```bash
# Test basic connectivity to all SBCs
python3 tests/e2e/test_sbc_connectivity.py

# Expected output:
# ✓ rpi4-manager (192.168.1.101) - SSH OK, Docker OK
# ✓ rpi4-worker1 (192.168.1.102) - SSH OK, Docker OK
# ✓ rpi4-worker2 (192.168.1.103) - SSH OK, Docker OK
# ✓ jetson-nano (192.168.1.104) - SSH OK, Docker OK, GPU OK
# ✓ nuc-mini (192.168.1.105) - SSH OK, Docker OK
```

### 2. Full System Test

```bash
# Run comprehensive multi-host test on real hardware
python3 tests/e2e/run_sbc_farm_test.py \
    --config tests/e2e/sbc_farm_config.yaml \
    --scenario full-system \
    --duration 30m
```

### 3. Continuous Testing

```bash
# Setup continuous testing (runs every 6 hours)
crontab -e

# Add this line:
0 */6 * * * cd /path/to/robocore-cli && python3 tests/e2e/run_sbc_farm_test.py --config tests/e2e/sbc_farm_config.yaml --scenario regression
```

### 4. GitHub Actions Self-Hosted Runner

```bash
# Setup self-hosted runner on coordinator machine
# 1. Go to GitHub repo Settings -> Actions -> Runners
# 2. Click "New self-hosted runner"
# 3. Follow setup instructions
# 4. Add labels: [self-hosted, sbc-farm, arm64, linux]

# Runner will execute nightly tests on real hardware
```

## Test Scenarios for SBC Farm

### Real Hardware Validation
```yaml
# tests/e2e/scenarios/sbc_real_hardware.yaml
name: "Real Hardware Validation"
description: "Test robocore-cli on actual SBC hardware"

phases:
  - name: "connectivity"
    tests:
      - ping_all_hosts
      - ssh_access_test
      - docker_daemon_test

  - name: "deployment"
    tests:
      - multi_host_deployment
      - cross_arch_builds
      - dds_discovery_test

  - name: "stress_test"
    tests:
      - high_cpu_workload
      - memory_pressure_test
      - network_bandwidth_test
      - container_restart_test

  - name: "endurance"
    duration: "24h"
    tests:
      - long_running_deployment
      - periodic_restart_test
      - memory_leak_detection
```

### Performance Benchmarking
```bash
# Benchmark robocore-cli performance on different hardware
python3 tests/performance/benchmark_sbc_farm.py \
    --duration 1h \
    --components 5 \
    --hosts 4 \
    --output benchmark_results.json
```

## Monitoring and Alerting

### System Metrics Collection
```python
# tests/e2e/sbc_farm/monitoring.py
import psutil
import prometheus_client

class SBCMonitor:
    def collect_metrics(self, sbc_config):
        """Collect CPU, memory, network, temperature metrics"""
        pass

    def check_health(self, sbc_config):
        """Health check for all SBCs"""
        pass

    def alert_on_failure(self, failure_type, sbc_name):
        """Send alerts via webhook/email"""
        pass
```

### Farm Dashboard
```html
<!-- Simple web dashboard to monitor SBC farm status -->
<!DOCTYPE html>
<html>
<head>
    <title>RoboCore SBC Farm Dashboard</title>
</head>
<body>
    <h1>SBC Test Farm Status</h1>
    <div id="farm-status">
        <!-- Real-time status of all SBCs -->
    </div>
</body>
</html>
```

This SBC farm setup provides a robust testing environment for validating robocore-cli across different hardware platforms and network configurations, ensuring reliability in real-world deployment scenarios.