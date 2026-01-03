# 🤖 robocore-cli

**robocore-cli** is a lightweight tool designed to simplify how you build, prototype, and deploy ROS 2 systems using containers.

It helps you structure your robot software as modular components, run them across one or more machines, and manage everything from a single configuration file. Whether you’re just wiring up your first prototype or preparing for deployment in the field, `robocore-cli` gives you a consistent workflow that grows with your project.


**robocore-cli** can help with:
- **Prototyping** quickly on your dev machine while keeping things reproducible
- **Deploying** to real robots without ad hoc scripts and broken dependencies
- **Iterating** on your robot system without needing to rebuild everything from scratch
- **Scaling** from a single board to a multi-machine setup
- **Sharing** complete robot recipes, not just isolated components

---

## 🧠 Philosophy

- **Deployment shouldn't be an afterthought**  
  Instead of hacking it together later, build with deployment in mind from the beginning.

- **Modular, reusable components**  
  Your robot is made up of logical building blocks (components) defined in a single YAML config.

- **One configuration = one robot**  
  All hosts, components, packages, and system structure live in a single declarative file: `robot.yaml`.

- **Native-speed dev, container-grade consistency**
  Enjoy the benefits of Docker without losing ROS tooling, hardware access, or real-time testing.

---

## Installation

### Prerequisites

- Python 3.10 or higher
- Docker
- pip

### Install from source

```bash
# Clone the repository
git clone https://github.com/your-org/robocore-cli.git
cd robocore-cli

# Create a virtual environment (recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install the package
pip install .
```

After installation, the `robocore` command will be available in your PATH:

```bash
robocore --help
```

### Development installation

For development, install in editable mode so changes take effect immediately:

```bash
pip install -e .
```

---
## 🛠️ How it works

Your robot is defined through a single YAML file that includes:
- **Configuration**: A file that defines what your robot is made of, where it runs, and how everything should be wired
- **Components**: ROS nodes grouped by purpose (e.g., perception, control, navigation)
- **Hosts**: Physical or virtual machines in your system
- **Common packages**: Shared ROS packages across components
- **Container settings**: Architecture, base images, registry info, devices, etc.

Then you follow a simple workflow:

1. **Stage**: Generate clean Docker images with your dependencies (but no user code)
2. **Build**: Compile each ROS workspace inside containers on your dev machine
3. **Sync**: Use smart diffs to transfer only changes to target hosts
4. **Iterate**: Make changes, test them, deploy again—fast
5. **Release**: Package and push versioned images for CI/CD or fleet rollout

It works like native ROS dev when you want speed, and like a deployment system when you need control.

---

## 💻 Platform Notes

###  macOS
- Extremely fast cross-compilation for ARM64 devices (Jetson, RPi5) thanks to native ARM support
- Docker runs inside a Linux VM, so **host networking is not available**
- Hybrid computation graphs (mixing local ROS nodes with containerized ones) are **not supported**
- GUI containers and simulations are **not supported**
- For local development with GUI tools, use [robostack](https://robostack.github.io/). If there's a `pixi.toml` in your project, the `pixi` command initializes a pixi shell with your robot's DDS configuration injected

### 🐧 Linux
- Fully supported with all features (host networking, GUI, hybrid graphs)
- ARM64 emulation on x86_64 is slow; set `build_on_device: true` on hosts to build natively on the target device
- NVIDIA GPU passthrough works natively with the `nvidia` component option

### WSL (Windows Subsystem for Linux)
- Simulations and GUI apps work (with WSLg)
- Hybrid computation graphs are **not supported** due to networking limitations

---

## ⚙️ Configuration
All of your robot’s components, hosts, deployment logic, and ROS metadata are defined in one centralized YAML file. This file is declarative, version-controllable, and reproducible.

### 1. Global Configuration

```yaml
# Required fields
ros_distro: humble                    # ROS 2 distribution (humble, iron, jazzy)
ros_domain_id: 0                      # ROS domain ID (0-232)
registry: docker.io/dragomirxyz      # Docker registry URL
image_prefix: myrobot                 # Docker image name prefix

# Optional fields
tag: latest                           # Docker image tag (default: latest)
build_dir: .robocore/build           # Build output directory
workspace_dir: ros_ws                 # ROS workspace directory name
mount_root: /home/user/ros_builds    # Remote mount root for builds

# Apt mirrors (for faster builds in certain regions)
apt_mirror: http://mirrors.example.com/ubuntu           # Custom Ubuntu apt mirror
ros_apt_mirror: http://mirrors.example.com/ros2/ubuntu  # Custom ROS apt mirror

# Global apt packages (installed in base image)
apt_packages:
  - vim
  - htop

# DDS configuration
enable_dds_router: false              # Enable DDS router container
discovery_server: localhost           # Discovery server address
```

**Required fields:**
* **ros_distro**: ROS 2 distribution (humble, iron, jazzy)
* **ros_domain_id**: Domain ID for DDS isolation (0-232)
* **registry**: Docker registry URL
* **image_prefix**: Prefix for generated image names

**Optional fields:**
* **tag**: Docker image tag (default: `latest`)
* **build_dir**: Local build output directory (default: `.robocore/build`)
* **workspace_dir**: ROS workspace name (default: `ros_ws`)
* **mount_root**: Remote directory for build artifacts
* **apt_mirror**: Custom Ubuntu apt mirror URL for faster downloads
* **ros_apt_mirror**: Custom ROS apt mirror URL
* **apt_packages**: System packages to install in base image
* **enable_dds_router**: Enable DDS router container
* **discovery_server**: DDS discovery server address

### 2. Hosts - Define the Machines in Your System

```yaml
hosts:
  - name: rpi5
    ip: pi5.local
    user: user
    arch: arm64
    port: 2375                        # Docker daemon port (default: 2375)
    manager: true                     # DDS discovery server manager role
    dds_ip: 192.168.1.100            # Secondary IP for DDS (defaults to ip)
    mount_root: /home/user/builds    # Override global mount_root for this host
    build_on_device: true            # Build on device (faster than emulation)
```

* **name**: Unique identifier for this host
* **ip**: Hostname or IP address for deployment
* **user**: SSH username for rsync and remote actions
* **arch**: Target architecture (amd64, arm64, armv7)
* **port**: Docker daemon TCP port (default: 2375)
* **manager**: Set to `true` to designate this host as the DDS discovery server
* **dds_ip**: Alternative IP for DDS communication (useful for multi-NIC setups)
* **mount_root**: Per-host override for build artifact directory
* **build_on_device**: Set to `true` to build components on this device instead of locally (faster for cross-arch builds)


### 3. Common packages
Define any packages or libraries that should be available in all components. You can use either VCS-based repositories or local source folders:

```yaml
common_packages:
  # VCS-based package (one or more repositories)
  - name: my_msgs
    repositories:
      - url: https://github.com/you/my_msgs.git
        version: main
        # folder is optional - auto-derived from URL if not specified
      - url: https://github.com/you/my_msgs_extras.git
        version: develop

  # Local source package (one or more folders)
  - name: leremix_control
    source:
      - ros_ws/leremix_control
      - ros_ws/leremix_control_plugin
      - ros_ws/leremix_servo_manager
```
These are included in the base image so they don't get duplicated per component—saving build time and storage.

### 4. Components – Modular, Swappable Units

Robocore-cli supports three ways to define components:

#### a. Robocore-Managed Build (Default)

robocore-cli generates a Dockerfile and manages the build process:

```yaml
components:
  - name: micro_ros_agent
    source: components/micro_ros_agent/ros_ws/src
    runs_on: rpi5
    apt_packages: []
    repositories:
      - url: https://github.com/micro-ROS/micro_ros_setup
        version: humble
        folder: micro_ros_agent
    postinstall:
      - "ros2 run micro_ros_setup create_agent_ws.sh"
      - "ros2 run micro_ros_setup build_agent.sh"
    entrypoint: ros2 run micro_ros_agent micro_ros_agent serial
    launch_args: "--dev /dev/ttyUSB0 -v6"
    devices:
      - "/dev/ttyUSB0:/dev/ttyUSB0"
```

**Source configuration:**
- **name**: Unique identifier for this component
- **source/sources**: Local source paths for ROS packages
- **runs_on**: Target host name (required)

**Dependencies:**
- **repositories**: VCS repositories in vcstool format
- **apt_packages**: System packages to install
- **pip_packages**: Python packages to install

**Runtime:**
- **entrypoint**: Container startup command
- **launch_args**: Arguments for entrypoint
- **environment**: Environment variables as key-value pairs

**Hardware:**
- **devices**: Device mappings (e.g., `/dev/ttyUSB0:/dev/ttyUSB0`)
- **ports**: Port mappings (e.g., `8080:80`)

**Build hooks:**
- **preinstall**: Commands to run before installing dependencies
- **postinstall**: Commands to run after installing dependencies

**Container options:**
- **privileged**: Run in privileged mode (default: false)
- **runtime**: Container runtime (e.g., `nvidia`)
- **nvidia**: Enable NVIDIA GPU support (default: false)
- **gpu_count**: Number of GPUs to allocate
- **gpu_device_ids**: Specific GPU device IDs
- **volumes**: Additional volume mounts
- **stdin_open**: Keep stdin open (default: false)
- **tty**: Allocate pseudo-TTY (default: false)

**Performance:**
- **shm_size**: Shared memory size (e.g., `2g`)
- **ipc_mode**: IPC mode (e.g., `host`)
- **cpuset**: CPU pinning (e.g., `0-3`)

#### b. External Pre-Built Image

Use existing Docker images from registries without any build:

```yaml
components:
  - name: rosboard
    image: thehale/rosboard:latest
    runs_on: rpi5
    ports:
      - "8888:8888"
    entrypoint: rosboard
```

- **image**: Full Docker image reference (registry/name:tag)
- Cannot be combined with `repositories`, `apt_packages`, or `pip_packages`

#### c. Custom Dockerfile Build

Build from your own Dockerfile for full control (GPU drivers, complex builds, etc.):

```yaml
components:
  - name: camera_driver
    build: docker/camera
    runs_on: orin
    devices:
      - "/dev/video0:/dev/video0"
    entrypoint: ros2 launch camera driver.launch.py
```

- **build**: Path to directory containing your Dockerfile (relative to project root)
- Cannot be combined with `repositories`, `apt_packages`, or `pip_packages`

See [docs/external-images.md](docs/external-images.md) for detailed examples and migration guides.

---

**Component Build Behavior**:
- For **robocore-managed** builds: compilation occurs during the `build` step
- For **external images**: no build step, image used as-is
- For **custom Dockerfiles**: build happens during `stage` step

---

## Commands

### init

**Command**: `robocore-cli <project_folder> init`

**Description**: Bootstrap a new project in the specified directory.

---

### prep

**Command**: `robocore-cli <project_folder> prep`

**Description**: Creates a base image with the **ros_distro** and including the common_packages. Make sure you re-run this everytime you add a new common package. 

---

### stage

**Command**: `robocore-cli <project_folder> stage`

**Description**: The stage step is where each component's container image is prepared with all its required dependencies, but without compiling or adding the actual application code yet. During this step, for every package:

* The base image is checked for changes and rebuilt only if needed (using hash-based caching).
* Any system dependencies (like via apt) and ROS packages from remote repositories (defined via VCS, e.g., Git) are installed into the container.
* The container ends up with a fully configured environment, ready for building the component's code in a consistent, reproducible way.
* The `docker-compose.yaml` files for each host get updated with the new containers and their settings.

For components using `image` or `build` fields, the stage step handles their Docker builds or skips building entirely for external images.

**Flags**
- `--refresh`: Just refreshes the `docker-compose.yaml` files, skipping any build step
- `--force-base`: Force rebuild of the base image even if cached (useful after manual changes)
- `-c, --component <name>`: Stage only the specified component
---

### build

**Command**:  `robocore-cli <project_folder> build`

**Description**:  The build step compiles the actual ROS workspace for each component using the environment prepared during the stage step. This process happens locally, inside the corresponding container, ensuring that builds are isolated, repeatable, and platform-appropriate. During this step, for every component:
* The corresponding stage container is started locally.
* The component's workspace is mounted into the container from `.robocore/build/{component}/ros_ws/`.
* The workspace is built using colcon, with output artifacts written to the install/ and build/ folders within the managed workspace.
* These artifacts are not embedded into the image, keeping the final runtime image clean and enabling fast rebuilds.

**Flags**
- `--component <name>`: Build only the specified component.

---

### launch

**Command**: `robocore-cli <project_folder> launch`

**Description**: Syncs compiled components to target hosts and launches containers. This command transfers only changed files using rsync (for remote hosts) and starts the appropriate containers on each host.

**Flags**:
- `--host <name>`: Only launch on this specific host (name from config file).

---

## Visualisation
ROS development often requires GUI tools like rviz2, rqt, or Gazebo. Depending on your setup, you can either run these tools directly on the robot and view them remotely via VNC, or run them locally on your development machine inside the correct container environment. Both workflows are supported and help maintain consistency between development and deployment.


### 1. Run the Desktop Environment on the robot and use VNC

**How it works:**
* A container with OpenGL and virtual display support is launched on the robot. I personally like to use (Tiryoh's docker-ros2-desktop-vnc image)[https://github.com/Tiryoh/docker-ros2-desktop-vnc]. 
* A lightweight VNC server is exposed on a configurable port.
* You connect using a web browser (e.g., http://robot-ip:6080) or a VNC client.

**When to use it:**
* Your robot either has enough processing power or your visualisations are simple (TF tree, 2D maps, etc)
* You want to monitor runtime visualization from any device (eg: Phone, iPad, SteamDeck) without local setup.

```yaml
hosts:
  - name: rpi5
    ip: pi5.local
    user: user
    arch: arm64

components:
  - name: camera
    folder: components/camera
    runs_on: rpi5 
    entrypoint: ros2 launch camera_bringup bringup.launch.py
    devices:
      - "/dev:/dev"

  - name: vnc
    image: ghcr.io/tiryoh/ros2-desktop-vnc:humble
    runs_on: rpi5
    ports:
      - "6080:80"
```


### 2.  Run the Desktop Environment on the Development Machine 
Instead of running GUI tools directly on the robot, you can also run them locally on your development machine using the same containerized environment—with the added benefit of better graphics performance and lower latency.

To do this, simply add your development machine as a host in your robot.yaml file. For example:

```yaml
hosts:
  - name: rpi5
    ip: pi5.local
    user: user
    arch: arm64

  - name: devmachine
    ip: 127.0.0.1
    user: your_username
    arch: amd64
```

And configure the VNC image to run on the devmachine, and if you select the `nvidia` flag as well, it will use your local GPU. 
```
components:
  - name: camera
    folder: components/camera
    runs_on: rpi5 
    entrypoint: ros2 launch camera_bringup bringup.launch.py
    devices:
      - "/dev:/dev"

  - name: vnc
    image: ghcr.io/tiryoh/ros2-desktop-vnc:humble
    runs_on: devmachine
    nvidia: true
    ports:
      - "6080:80"
```

**When to use it:**
* You need high-performance visualization tools like rviz2, but the robot’s hardware is limited.
* You’re debugging or inspecting data in a hybrid robot+desktop computation graph.
* You want to work locally without modifying container images for GUI support.

This approach however might quickly saturate your bandwidth, introducing latency in the whole computation graph, so be mindful of this tradeoff between graphical fidelity and network usage. 

---

## Networking
`robocore-cli` is designed to support robust, multi-host ROS 2 deployments by building on the DDS discovery protocol, using a structured yet flexible approach to communication between containers and physical machines. he goal is to make multi-machine setups (like robot + workstation or multiple hosts running on the same system) behave as a single unified ROS 2 system, without requiring custom bridging, remapping, or brittle configuration hacks.

By default, all containers launched by robocore-cli use host networking. This simplifies ROS 2 communication because:
* DDS (specifically Fast DDS or Cyclone DDS) can directly use multicast, broadcast, and unicast to discover and connect nodes.
* No need to expose individual container ports or deal with NAT and Docker’s virtual networks.
* Tools like `ros2 topic echo`, `rviz2`, or `ros2 node list` work out of the box across hosts—assuming firewalls and routes are properly configured.

To avoid the limitations of peer-to-peer discovery at scale or in mixed-network setups, robocore-cli automatically provisions a Fast DDS Discovery Server container per robot network.

In this setup:
* A discovery server container is launched on the robot or designated control node.
* All other containers on the same host (and on other hosts) act as superclients and register with this server.
* This centralizes discovery, reduces multicast traffic, and allows fine-grained control over discovery domains.

Long story short, as long as your hosts are defined corectly in the config.yaml file, and are on the same subnet, the sky (and network bandwidth) is the limit. During development we reccomend having all of the hosts connected by Ethernet or Fast WiFi such as 6E or 7.

---

## Multi-host systems

One of the most powerful capabilities of the `robocore-cli` workflow is support for multi-host systems where different parts of your ROS 2 stack run on different physical machines but function as a single, seamless graph.


By defining multiple `hosts` in your `robot.yaml` and assigning components to them via the `runs_on` field, `robocore-cli` enables developers to shift computation between machines with minimal effort. All networking, DDS discovery, and build architecture targeting are handled for you.


### ✅ **Example 1: MoveIt or Navigation Stack Initially on Dev Machine, Then Moved to Robot**

During early development, you might want to run CPU- or GPU-intensive components like MoveIt, SLAM, or the Nav2 stack on your development machine for faster iteration, easier debugging, and better performance.

```yaml
components:
  - name: moveit_planner
    runs_on: devmachine
```

Later, when it’s ready for production deployment, you can move the same component to the robot by simply changing the runs_on field:

```yaml
components:
  - name: moveit_planner
    runs_on: raspberrypi5
```

Then re-run:
```yaml
robocore-cli stage
robocore-cli build
robocore-cli launch
```

The tool will:
* Automatically rebuild the component for the correct target architecture (e.g., arm64 vs. amd64).
* Sync the compiled artifacts to the robot.
* Integrate it into the same ROS 2 domain and DDS discovery graph.

No code changes or manual docker fiddling required—just edit a config file and go.



### ✅ **Example 2: Distributed Perception Stack Across Multiple Machines**
Imagine a robot equipped with several cameras positioned around its body—for example, front, side, and rear views. Due to the computational cost of running modern deep learning inference on multiple streams, the cameras are each connected to separate Jetson devices mounted onboard. These Jetsons handle preprocessing and inference locally to reduce network and CPU load on the main computer.

Meanwhile, a central main computer (e.g., an x86 SBC or industrial PC) acts as the coordination hub—receiving processed data from all Jetsons, fusing it, and making high-level decisions.

This setup can be easily modeled in robocore-cli:

```yaml
hosts:
  - name: main_computer
    ip: 192.168.1.100
    user: robot
    arch: amd64

  - name: jetson_front
    ip: 192.168.1.101
    user: robot
    arch: arm64

  - name: jetson_side
    ip: 192.168.1.102
    user: robot
    arch: arm64

  - name: jetson_rear
    ip: 192.168.1.103
    user: robot
    arch: arm64

components:
  - name: front_inference
    runs_on: jetson_front
    folder: components/perception_front

  - name: side_inference
    runs_on: jetson_side
    folder: components/perception_side

  - name: rear_inference
    runs_on: jetson_rear
    folder: components/perception_rear

  - name: perception_aggregator
    runs_on: main_computer
    folder: components/perception_aggregator
```

Advantages of this setup:
* Each Jetson processes data close to the sensor, reducing latency and I/O load.
* The main computer receives compact, semantically rich data (e.g., object bounding boxes or segmentation maps), not raw images.
* All components participate in a unified ROS 2 graph, coordinated by the central DDS Discovery Server.

With robocore-cli, the complexity of building for mixed architectures (x86 for the main computer, ARM64 for Jetsons), syncing code, and managing the container network is abstracted away. You simply define the roles in the robot.yaml, and the tool handles staging, building, syncing, and orchestration.


This approach scales well for robots with multi-modal perception systems and provides a clean separation of concerns across devices.

---

## 💡 Notes & Tips
Here are a few quality of life tips that are not covered or automatically handled by `robocore-cli`. 

### Enabling TCP socket for docker hosts.
robocore-cli aims to be as transparent as possible to the systems installed on the robots. There is only one modification required for hosts to be able to work with this tool, and that is to have the docker host on the clients listen on a TCP socket. By default we are assuming the network you are running this on is under your control, and it is safe to leave the socket unprotected. For more information about how to secure the Docker TCP check out [this guide](). 

1. Create `daemon.json` file in `/etc/docker`:

```
"hosts": ["tcp://0.0.0.0:2375", "unix:///var/run/docker.sock"]}
```

2. Add `/etc/systemd/system/docker.service.d/override.conf`
```
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd
```

3. Reload the systemd daemon:
```systemctl daemon-reload```

4. Restart docker:
```systemctl restart docker.service```

