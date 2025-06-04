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

## ⚙️ Configuration
All of your robot’s components, hosts, deployment logic, and ROS metadata are defined in one centralized YAML file. This file is declarative, version-controllable, and reproducible.

### 1. Basics 


```yaml

ros_distro: humble
ros_domain_id: 0
registry: docker.io/dragomirxyz
image_prefix: lekiwiv2-arm64
```

* **ros_distro**: Which ROS 2 distribution you're targeting (e.g., humble, iron)
* **ros_domain_id**: For isolating DDS domains in multi-robot systems
* **registry**: Your Docker registry, local or remote
* **image_prefix**: Namespacing for generated container images

### 2. Hosts - Define the Machines in Your System

```yaml
hosts:
  - name: rpi5
    ip: pi5.local
    user: user
    arch: arm64
```

* **name:** Internal reference for components
* **ip:** Hostname or IP for deployment
* **user:** SSH username for rsync and remote actions
* **arch:** Architecture used for container cross-builds (amd64, arm64, etc.)


### 3. Common packages
Define any packages or libraries that should be available in all components:

```yaml
common_packages:
  - name: my_msgs
    folder: common_packages/my_msgs
    repo: https://github.com/you/my_msgs.git
    branch: main
```
These are included in the base image so they don’t get duplicated per component—saving build time and storage.

### 4. Components – Modular, Swappable Units

```yaml
components:
  - name: micro_ros_agent
    folder: components/micro_ros_agent
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

- **name**: The unique identifier for this component 
- **folder**: Where its ROS 2 workspace lives 
- **runs_on**: Which host this component deploys to
- **entrypoint**: What command runs at container start **(optional)**
- **devices**: Which devices to expose (e.g., /dev/ttyUSB0) **(optional)**
- **apt_packages**: List of apt packages you want to install **(optional)**
- **repositories**: Define dependencies declaratively using the standard vcstool format **(optional)**
- **postinstall**: Commands to run after installing dependencies. **(optional)**


You can mix and match different types of package sources within a container; however, we do not recommend combining VCS-based packages with apt-based packages in the same component, as this can lead to confusion and maintenance issues down the line.

For components that use only apt packages or VCS repositories, compilation will occur during the stage step. If a component includes a local workspace, it will be built during the build step.

Additionally, a component can be based on a pre-packaged ROS image. In this case, the folder, apt_packages, and repositories fields should not be used—if any of these are specified alongside a base image, it will result in an error.

---

## Commands
The command set of `robocore-cli` is deliberately 

### init
**Command**: `robocore-cli <project_folder> init`
**Description**: Bootstrap a new project in the specified directory.

### prepare-base
**Command**: `robocore-cli <project_folder> prepare-base`
**Description**: Creates a base image with the **ros_distro** and including the common_packages. Make sure you re-run this everytime you add a new common package. 

### stage
**Command**: `robocore-cli <project_folder> stage`
**Description**: The stage step is where each component's container image is prepared with all its required dependencies, but without compiling or adding the actual application code yet. During this step, for every package:

* The base image (e.g., a minimal ROS image) is pulled.
* Any system dependencies (like via apt) and ROS packages from remote repositories (defined via VCS, e.g., Git) are installed into the container.
* The container ends up with a fully configured environment, ready for building the component’s code in a consistent, reproducible way.
* The `docker-compose.yaml` files for each host get updated with the new containers and their settings. 

**Flags**
- `--refresh`: Just refreshes the `docker-compose.yaml` files, skipping any build step

### build
**Command**:  `robocore-cli <project_folder> build`
**Description**:  The build step compiles the actual ROS workspace for each component using the environment prepared during the stage step. This process happens locally, inside the corresponding container, ensuring that builds are isolated, repeatable, and platform-appropriate. During this step, for every component:
* The corresponding stage container is started locally.
* The component’s local ROS workspace (ros_ws) is mounted into the container.
* The workspace is built using colcon, with output artifacts written to the install/ and build/ folders on the host.
* These artifacts are not embedded into the image, keeping the final runtime image clean and enabling fast rebuilds.

**Flags**
- `--component <name>`: Build only the specified component.

### sync
**Command**: `robocore-cli <project_folder> sync`
**Description**:  The sync step transfers the compiled components and runtime files from the local machine to the target robot(s). It ensures the robot runs the latest code without needing to rebuild anything on the device, making iteration fast and deployment lightweight. During this step, for every component assigned to a host:

* The install tree (built during the build step) and any necessary runtime files are prepared for transfer.
* Using rsync, only changed files are copied to the target robot, minimizing bandwidth and time.

### run 
**Command**: `robocore-cli <project_folder> run`
**Description**: This command connects to the docker daemon on each one of the hosts and launches the appropiate containers.  
**Flags**:
- `--host <name>`: There are siuations where we only want to start/restart the containers on one of the hosts. 

### shell
**Command**: `robocore-cli <project_folder> shell --component <name>`
**Description**: Launches an interactive local development shell inside a container for the specified component. This is ideal for generating packages, running ROS tools, or experimenting within a fully provisioned ROS environment that mirrors your deployment image.
* A local container is started using the component's staged image.
* The component’s source folder (e.g., components/<name>) is mounted into the container.
* The full ROS environment is sourced and ready for immediate use.
* This shell runs entirely on your development machine and does not affect or interact with any deployed robot containers.

**Flags**:
- `--component <name>`: The component to open the local shell for (required).

### connect
**Command**: `robocore-cli <project_folder> connect --component <name>`
**Description**: Opens an interactive shell inside the running container of a specified component on a remote robot. This is useful for debugging, inspecting runtime logs, or manually testing commands in the live environment. During this step:
* An interactive shell is started inside the live container on the host it belongs to, with ROS fully sourced.
* This shell connects directly to the deployed robot environment and is best used for runtime introspection—not for editing code or modifying container contents persistently.

**Flags**:
- `--component <name>`: The name of the component to connect to (required).


---

## Visualisation
ROS development often requires GUI tools like rviz2, rqt, or Gazebo. Depending on your setup, you can either run these tools directly on the robot and view them remotely via VNC, or run them locally on your development machine inside the correct container environment. Both workflows are supported and help maintain consistency between development and deployment.

### Run the Desktop Environment on the robot and use VNC

![](https://private-user-images.githubusercontent.com/3256629/399220117-137a5272-f6a3-490f-8bfc-168d082ac949.gif?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NDkwNDI5MzcsIm5iZiI6MTc0OTA0MjYzNywicGF0aCI6Ii8zMjU2NjI5LzM5OTIyMDExNy0xMzdhNTI3Mi1mNmEzLTQ5MGYtOGJmYy0xNjhkMDgyYWM5NDkuZ2lmP1gtQW16LUFsZ29yaXRobT1BV1M0LUhNQUMtU0hBMjU2JlgtQW16LUNyZWRlbnRpYWw9QUtJQVZDT0RZTFNBNTNQUUs0WkElMkYyMDI1MDYwNCUyRnVzLWVhc3QtMSUyRnMzJTJGYXdzNF9yZXF1ZXN0JlgtQW16LURhdGU9MjAyNTA2MDRUMTMxMDM3WiZYLUFtei1FeHBpcmVzPTMwMCZYLUFtei1TaWduYXR1cmU9MjNkNzY2NjlhYmMyYzdhY2I3OGMwZTNhZTNiMjFjOWM0NmMxZjNmNTUzMjE5MzJhYTZlYTZhZTVhZWNmYmYyNyZYLUFtei1TaWduZWRIZWFkZXJzPWhvc3QifQ.rz1HAWD7UsSXmNcOV64cl3lqfc6x43Olk92pscO-7fY)

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

### Run the Desktop Environment on the Development Machine 
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
Networking is at the core of any distributed ROS 2 system. `robocore-cli` is designed to support robust, multi-host ROS 2 deployments by building on the DDS discovery protocol, using a structured yet flexible approach to communication between containers and physical machines.
The goal is to make multi-machine setups (like robot + workstation or multiple hosts running on the same system) behave as a single unified ROS 2 system, without requiring custom bridging, remapping, or brittle configuration hacks.

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

### Multi-host systems

One of the most powerful capabilities of the `robocore-cli` workflow is support for multi-host systems where different parts of your ROS 2 stack run on different physical machines but function as a single, seamless graph.
By defining multiple `hosts` in your `robot.yaml` and assigning components to them via the `runs_on` field, `robocore-cli` enables developers to shift computation between machines with minimal effort. All networking, DDS discovery, and build architecture targeting are handled for you.

#### ✅ **Example 1: MoveIt or Navigation Stack Initially on Dev Machine, Then Moved to Robot**

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
robocore-cli sync
```

The tool will:
* Automatically rebuild the component for the correct target architecture (e.g., arm64 vs. amd64).
* Sync the compiled artifacts to the robot.
* Integrate it into the same ROS 2 domain and DDS discovery graph.

No code changes or manual docker fiddling required—just edit a config file and go.

#### ✅ **Example 2: Distributed Perception Stack Across Multiple Machines**
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
        {"hosts": ["tcp://0.0.0.0:2375", "unix:///var/run/docker.sock"]}
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


### 🔐 SSH Keys

When working with multiple robots or development machines, repeatedly entering SSH passwords can quickly become a bottleneck—especially during `robocore-cli sync`, `connect`, or remote `shell` sessions.
Setting up SSH key-based authentication eliminates the need for manual password entry and allows secure, unattended automation.


#### ✅ Generate an SSH Key (if you don't have one)

Check for an existing key:
```bash
ls ~/.ssh/id_rsa.pub
```

If the file doesn't exist, generate one:

```bash
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

Just press Enter to accept the defaults. This will create:
* `~/.ssh/id_rsa` — your private key (keep this safe!)
* `~/.ssh/id_rsa.pub` — your public key (can be shared)


#### ✅ Copy Your SSH Key to Each Remote Host
Use ssh-copy-id to copy your public key to a remote host:

```ssh-copy-id robot@192.168.1.101```


Repeat for each host listed in your robot.yaml config. For example:

```bash
ssh-copy-id robot@pi5.local
ssh-copy-id robot@jetson.local
```

This will append your public key to the remote user's `~/.ssh/authorized_keys` file and set correct permissions.

### ⏰ Time Synchronisation
For DDS to function reliably across machines—and especially for ROS 2 tools that depend on timestamps (e.g., TF, RViz, and sensor fusion)—clock synchronization is critical.

robocore-cli assumes that:

* All hosts (robots and development machines) are synced via NTP or Chrony.
* The system clocks are within a tight tolerance (typically <1 second drift).

If clocks are not in sync, you may experience:

* Inconsistent TF transforms or warnings in rviz2
* rosbag2 recording incorrect timestamps
* Sensor fusion or SLAM systems behaving erratically

Time sync is not handled by robocore-cli directly, but it is a required infrastructure layer—especially in hybrid deployments. We recommend configuring NTP on all hosts, or using chrony for more accurate local sync in constrained networks.

### 🔧 Option 1: Use NTP (Network Time Protocol)

This is the most widely supported method.

```bash
sudo apt install ntp
```

Then edit `/etc/ntp.conf` (optional) to point to your local or public NTP servers.

Start and enable the service:

```baah
sudo systemctl enable ntp
sudo systemctl start ntp
```

To check sync status:

```
ntpq -p
```

### 🔧 Option 2: Use Chrony (for faster convergence & offline setups)

Install crhony on all machines: 
```bash
sudo apt install chrony
```

#### On your main reference machine 

1. Edit /etc/chrony/chrony.conf:

```
# Comment out public servers
# pool ntp.ubuntu.com ...

# Allow your robot subnet
allow 192.168.1.0/24

# Act as a time source
local stratum 10
```

2. Restart the crhony service:
`sudo systemctl restart chrony`

#### On your client machines

1. Edit /etc/chrony/chrony.conf and add:
`server 192.168.1.100 iburst   # IP of your main reference machine`

2. Restart the crhony service:
`sudo systemctl restart chrony`