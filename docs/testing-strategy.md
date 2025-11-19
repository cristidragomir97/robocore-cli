# End-to-End Testing Strategy

This document outlines a comprehensive testing strategy for robocore-cli, covering multi-host deployment, DDS discovery, cross-architecture builds, and real ROS communication scenarios.

## Testing Objectives

### Primary Goals
1. **Validate complete deployment workflow** from `init` to running containers
2. **Test multi-host scenarios** with real network communication
3. **Verify DDS discovery and ROS communication** across hosts
4. **Ensure cross-architecture compatibility** (amd64 ↔ arm64)
5. **Test workspace management** (legacy vs new format)
6. **Validate error handling and recovery** scenarios

### Secondary Goals
1. **Performance benchmarking** of build and deployment times
2. **Resource usage monitoring** (CPU, memory, network)
3. **Scalability testing** with increasing number of hosts/components
4. **Integration testing** with real ROS packages and applications

## Testing Infrastructure

### Architecture Overview

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Test Runner   │    │  GitHub Actions │    │   SBC Farm      │
│   (Orchestrator)│    │   (CI/CD)       │    │  (Real HW)      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
         ┌───────────────────────┼───────────────────────┐
         │                       │                       │
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Local Vagrant │    │  Docker-in-VM   │    │   Physical SBCs │
│   Multi-VM      │    │   (GitHub)      │    │   (arm64/amd64) │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Infrastructure Components

#### 1. Local Development Testing (Vagrant)
```ruby
# Vagrantfile for multi-VM testing
Vagrant.configure("2") do |config|
  # Manager node (DDS discovery server)
  config.vm.define "manager" do |manager|
    manager.vm.box = "ubuntu/jammy64"
    manager.vm.network "private_network", ip: "192.168.56.10"
    manager.vm.provider "virtualbox" do |v|
      v.memory = 2048
      v.cpus = 2
    end
  end

  # Worker nodes
  (1..3).each do |i|
    config.vm.define "worker#{i}" do |worker|
      worker.vm.box = "ubuntu/jammy64"
      worker.vm.network "private_network", ip: "192.168.56.#{10+i}"
      worker.vm.provider "virtualbox" do |v|
        v.memory = 1024
        v.cpus = 1
      end
    end
  end

  # ARM64 emulation node
  config.vm.define "arm64" do |arm|
    arm.vm.box = "ubuntu/jammy64"
    arm.vm.network "private_network", ip: "192.168.56.20"
    arm.vm.provider "virtualbox" do |v|
      v.memory = 2048
      v.cpus = 2
    end
  end
end
```

#### 2. GitHub Actions CI/CD
```yaml
# .github/workflows/e2e-tests.yml
name: End-to-End Tests

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  multi-vm-test:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        scenario: [single-host, multi-host, cross-arch]

    steps:
      - uses: actions/checkout@v4

      - name: Setup Docker
        uses: docker/setup-buildx-action@v3

      - name: Create test network
        run: docker network create robocore-test

      - name: Run test scenario
        run: ./tests/e2e/run-scenario.sh ${{ matrix.scenario }}
```

#### 3. SBC Test Farm Integration
```yaml
# Self-hosted runner configuration
name: SBC Farm Tests
on:
  schedule:
    - cron: '0 2 * * *'  # Nightly tests

jobs:
  real-hardware-test:
    runs-on: [self-hosted, sbc-farm]
    strategy:
      matrix:
        arch: [amd64, arm64]
        scenario: [perception, navigation, full-stack]
```

## Test Scenarios

### Scenario 1: Single Host Deployment

**Objective:** Validate basic robocore-cli functionality on a single machine

**Setup:**
```yaml
# test-configs/single-host.yaml
ros_distro: humble
ros_domain_id: 42
registry: localhost:5000
image_prefix: test_robot

hosts:
  - name: localhost
    ip: 127.0.0.1
    user: testuser
    arch: amd64
    manager: true

components:
  - name: test_node
    source: test-packages/simple_publisher
    runs_on: localhost
    entrypoint: ros2 run simple_publisher publisher
```

**Test Steps:**
1. `robocore-cli init test-project`
2. Copy test configuration
3. `robocore-cli stage`
4. `robocore-cli build`
5. `robocore-cli deploy`
6. Verify container is running
7. Check ROS topics are published

### Scenario 2: Multi-Host Deployment

**Objective:** Test deployment across multiple hosts with DDS discovery

**Setup:**
```yaml
# test-configs/multi-host.yaml
ros_distro: humble
ros_domain_id: 42
registry: registry.test.local:5000
image_prefix: multi_robot
enable_dds_router: true
discovery_server: 192.168.56.10

hosts:
  - name: manager
    ip: 192.168.56.10
    user: vagrant
    arch: amd64
    manager: true

  - name: worker1
    ip: 192.168.56.11
    user: vagrant
    arch: amd64

  - name: worker2
    ip: 192.168.56.12
    user: vagrant
    arch: amd64

components:
  - name: publisher
    source: test-packages/multi_publisher
    runs_on: worker1
    entrypoint: ros2 run multi_publisher publisher

  - name: subscriber
    source: test-packages/multi_subscriber
    runs_on: worker2
    entrypoint: ros2 run multi_subscriber subscriber

  - name: dds_router
    repositories:
      - url: https://github.com/eProsima/DDS-Router.git
        version: main
        folder: dds_router
    runs_on: manager
```

**Test Steps:**
1. Provision multi-VM environment
2. Deploy robocore-cli configuration
3. Verify DDS discovery between hosts
4. Test ROS topic communication across network
5. Simulate network partitions and recovery
6. Verify graceful shutdown

### Scenario 3: Cross-Architecture Deployment

**Objective:** Test amd64 ↔ arm64 communication and builds

**Setup:**
```yaml
# test-configs/cross-arch.yaml
hosts:
  - name: amd64_host
    ip: 192.168.56.10
    user: vagrant
    arch: amd64
    manager: true

  - name: arm64_host
    ip: 192.168.56.20
    user: vagrant
    arch: arm64

components:
  - name: amd64_node
    sources:
      - test-packages/sensor_driver
      - test-packages/data_processor
    runs_on: amd64_host

  - name: arm64_node
    source: test-packages/edge_controller
    runs_on: arm64_host
```

### Scenario 4: Complex Multi-Component System

**Objective:** Test realistic robotics application with multiple interconnected components

**Setup:**
```yaml
# test-configs/full-system.yaml
components:
  # Perception system
  - name: perception
    sources:
      - test-packages/camera_driver
      - test-packages/lidar_processor
      - test-packages/perception_fusion
    runs_on: perception_unit
    devices:
      - "/dev/video0:/dev/video0"

  # Navigation system
  - name: navigation
    source: test-packages/nav_stack
    runs_on: main_compute

  # Control system
  - name: control
    sources:
      - test-packages/motor_controller
      - test-packages/safety_monitor
    runs_on: control_unit
    devices:
      - "/dev/ttyACM0:/dev/ttyACM0"

  # Simulation component
  - name: simulation
    image: osrf/ros:humble-desktop
    simulate: true
    runs_on: main_compute
```

## Test Implementation

### Test Framework Structure

```
tests/
├── e2e/
│   ├── scenarios/
│   │   ├── single_host.py
│   │   ├── multi_host.py
│   │   ├── cross_arch.py
│   │   └── full_system.py
│   ├── fixtures/
│   │   ├── test_packages/
│   │   ├── configs/
│   │   └── docker/
│   ├── utils/
│   │   ├── vm_manager.py
│   │   ├── network_utils.py
│   │   ├── ros_utils.py
│   │   └── docker_utils.py
│   ├── conftest.py
│   └── test_runner.py
├── integration/
├── unit/
└── performance/
```

### Test Base Classes

```python
# tests/e2e/base.py
import pytest
import subprocess
import time
from pathlib import Path
from typing import List, Dict, Any

class RobocoreE2ETest:
    """Base class for end-to-end tests"""

    def __init__(self, config_file: str, test_name: str):
        self.config_file = config_file
        self.test_name = test_name
        self.project_dir = None
        self.cleanup_commands = []

    def setup_test_environment(self):
        """Setup test project and environment"""
        # Create temporary project directory
        # Copy test configuration
        # Setup test packages
        pass

    def teardown_test_environment(self):
        """Cleanup test environment"""
        # Stop containers
        # Remove test images
        # Clean project directory
        pass

    def run_robocore_command(self, command: List[str]) -> subprocess.CompletedProcess:
        """Run robocore-cli command with proper error handling"""
        pass

    def wait_for_container_ready(self, container_name: str, timeout: int = 60):
        """Wait for container to be ready"""
        pass

    def verify_ros_communication(self, topic: str, message_type: str, timeout: int = 30):
        """Verify ROS topic communication across hosts"""
        pass

    def verify_dds_discovery(self, expected_participants: List[str]):
        """Verify DDS discovery is working"""
        pass
```

### Specific Test Implementations

```python
# tests/e2e/scenarios/multi_host.py
class TestMultiHostDeployment(RobocoreE2ETest):

    def test_multi_host_deployment(self):
        """Test complete multi-host deployment workflow"""

        # Setup
        self.setup_test_environment()

        try:
            # Stage components
            result = self.run_robocore_command(['stage'])
            assert result.returncode == 0

            # Build components
            result = self.run_robocore_command(['build'])
            assert result.returncode == 0

            # Deploy to hosts
            result = self.run_robocore_command(['deploy'])
            assert result.returncode == 0

            # Start containers
            result = self.run_robocore_command(['run'])
            assert result.returncode == 0

            # Wait for all containers to be ready
            self.wait_for_container_ready('publisher')
            self.wait_for_container_ready('subscriber')

            # Verify DDS discovery
            self.verify_dds_discovery(['publisher', 'subscriber'])

            # Verify ROS communication
            self.verify_ros_communication('/test_topic', 'std_msgs/String')

            # Test component restart
            self.restart_component('publisher')
            self.verify_ros_communication('/test_topic', 'std_msgs/String')

        finally:
            self.teardown_test_environment()

    def test_network_partition_recovery(self):
        """Test behavior during network partitions"""
        # Simulate network partition
        # Verify graceful degradation
        # Restore network
        # Verify recovery
        pass
```

### VM and Infrastructure Management

```python
# tests/e2e/utils/vm_manager.py
class VagrantManager:
    """Manage Vagrant VMs for testing"""

    def __init__(self, vagrantfile_path: str):
        self.vagrantfile_path = vagrantfile_path
        self.active_vms = []

    def start_vms(self, vm_names: List[str] = None):
        """Start specified VMs or all VMs"""
        pass

    def stop_vms(self):
        """Stop all active VMs"""
        pass

    def get_vm_ip(self, vm_name: str) -> str:
        """Get IP address of specified VM"""
        pass

    def execute_on_vm(self, vm_name: str, command: str) -> str:
        """Execute command on specified VM"""
        pass

class DockerInVMManager:
    """Manage Docker containers within VMs"""

    def __init__(self, vm_manager: VagrantManager):
        self.vm_manager = vm_manager

    def setup_docker_on_vm(self, vm_name: str):
        """Install and configure Docker on VM"""
        pass

    def create_docker_network(self, network_name: str):
        """Create Docker network across VMs"""
        pass
```

### Network and ROS Testing Utilities

```python
# tests/e2e/utils/ros_utils.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROSTestNode(Node):
    """Test node for ROS communication verification"""

    def __init__(self):
        super().__init__('test_node')
        self.messages_received = []

    def create_test_publisher(self, topic: str, message_type):
        """Create publisher for testing"""
        pass

    def create_test_subscriber(self, topic: str, message_type):
        """Create subscriber for testing"""
        pass

    def wait_for_message(self, topic: str, timeout: float = 10.0):
        """Wait for message on specified topic"""
        pass

def verify_dds_participants(expected_participants: List[str], timeout: float = 30.0) -> bool:
    """Verify expected DDS participants are discovered"""
    # Use ros2 command line tools to check participants
    # Parse output and verify all expected participants are present
    pass

def check_topic_connectivity(publisher_host: str, subscriber_host: str,
                           topic: str, timeout: float = 30.0) -> bool:
    """Verify topic communication between hosts"""
    pass
```

## Test Execution Strategies

### 1. GitHub Actions Matrix

```yaml
strategy:
  matrix:
    test-type: [unit, integration, e2e-single, e2e-multi]
    ros-distro: [humble, iron]
    architecture: [amd64, arm64]
    include:
      - test-type: e2e-multi
        vm-count: 3
      - test-type: e2e-single
        vm-count: 1
```

### 2. Nightly SBC Farm Tests

```yaml
# .github/workflows/nightly-sbc.yml
name: Nightly SBC Farm Tests

on:
  schedule:
    - cron: '0 2 * * *'

jobs:
  sbc-performance-test:
    runs-on: [self-hosted, sbc-farm]
    steps:
      - name: Full System Test
        run: |
          cd tests/e2e/sbc_farm
          python run_full_system_test.py --duration=8h --iterations=10
```

### 3. Local Development Workflow

```bash
# Developer workflow
make test-local          # Run all local tests
make test-vm-single      # Single VM test
make test-vm-multi       # Multi VM test
make test-cross-arch     # Cross architecture test
make test-performance    # Performance benchmarks
```

## SBC Test Farm Setup

### Hardware Configuration
```yaml
# sbc-farm-config.yaml
farm_name: "home_sbc_farm"
coordinator_ip: "192.168.1.100"

nodes:
  - name: "rpi4-1"
    ip: "192.168.1.101"
    arch: "arm64"
    role: "manager"
    specs:
      cpu: "4x Cortex-A72"
      memory: "8GB"

  - name: "rpi4-2"
    ip: "192.168.1.102"
    arch: "arm64"
    role: "worker"

  - name: "jetson-nano"
    ip: "192.168.1.103"
    arch: "arm64"
    role: "worker"
    specs:
      gpu: "128-core Maxwell"

  - name: "nuc-mini"
    ip: "192.168.1.104"
    arch: "amd64"
    role: "worker"
```

### SBC Farm Management

```python
# tests/e2e/sbc_farm/farm_manager.py
class SBCFarmManager:
    """Manage SBC test farm"""

    def __init__(self, config_file: str):
        self.config = self.load_config(config_file)
        self.nodes = {}

    def provision_test_environment(self, test_scenario: str):
        """Provision specific test environment on SBCs"""
        pass

    def deploy_test_containers(self, config: dict):
        """Deploy test containers to SBCs"""
        pass

    def run_endurance_test(self, duration: str, iterations: int):
        """Run long-running endurance tests"""
        pass

    def collect_performance_metrics(self):
        """Collect CPU, memory, network metrics from all nodes"""
        pass
```

## Test Data and Fixtures

### Test ROS Packages

```
tests/e2e/fixtures/test-packages/
├── simple_publisher/
│   ├── package.xml
│   ├── setup.py
│   └── simple_publisher/
│       └── publisher.py
├── multi_subscriber/
├── sensor_simulator/
├── navigation_dummy/
└── cross_platform_node/
```

### Performance Benchmarks

```python
# tests/performance/benchmarks.py
class RobocorePerformanceBenchmarks:

    def benchmark_build_time(self, num_components: int):
        """Benchmark build time vs number of components"""
        pass

    def benchmark_deploy_time(self, num_hosts: int):
        """Benchmark deployment time vs number of hosts"""
        pass

    def benchmark_memory_usage(self, scenario: str):
        """Benchmark memory usage for different scenarios"""
        pass
```

This comprehensive testing strategy ensures robocore-cli works reliably across different environments, architectures, and deployment scenarios while providing both automated CI/CD testing and real hardware validation through your SBC farm.