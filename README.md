# robocore-cli 

## Overview

This CLI tool helps you **orchestrate ROS development** by:

1. **Parsing** your **YAML configuration** files (e.g., `robot.yaml`, optional includes like `hardware.yaml`, and `hosts.yaml`).
2. **Generating** a **local build environment** that organizes and fetches ROS packages.
3. **Creating** Dockerfiles automatically (based on detected dependencies and user-defined segments).
4. **Building** those Docker images **on each remote host** specified in `hosts.yaml`.

## Workflow

1. **Create** or **edit** `robot.yaml`, optionally referencing **`hardware.yaml`** for additional bricks.
2. **Create** `hosts.yaml` listing each remote machine’s IP, username, etc.
3. **Organize** local brick folders in `./bricks/<brick_id>/ros_ws` if you have local sources.
4. **Run** the CLI tool:
   ```bash
   robco --solution-dir /path/to/config --pull --clean
   ```
   - `--solution-dir` points to the directory with `robot.yaml` and `hosts.yaml`.
   - `--pull` updates GitHub repos if already cloned.
   - `--clean` removes existing Docker images on each host before building.
5. The tool creates a `build/` folder (per-host, per-brick subfolders), **detects dependencies**, **generates Dockerfiles**, and **remotely builds** the images on the specified hosts.

With this structure, each brick is **mapped** to the correct host, and all local files (Dockerfiles, sources) are created under `build/`. The final Docker images run in a consistent environment that matches your ROS version and optional overlay network.
"""

## Configuration Structure

#### 1. **`robot.yaml`**

Your **main** `robot.yaml`:

- **`robot:`** must be the top-level key.
- **`name:`** sets the robot’s name (e.g., `"stark"`).
- **`ros-version:`** sets the ROS 2 distro (e.g., `"humble"`).
- **`bricks:`** contains one or more named bricks or references to includes:
  - An **`includes`** list allows you to include external YAML files (e.g., `hardware.yaml`).
  - Each brick entry has:
    - `host` (which must match one of the hosts defined in `hosts.yaml`).
    - `package` details (type of source, optional `launch`, `preinstall`, etc.).
    - Zero or one **`endpoints`** block describing publishers/subscribers.

**Example excerpt** (simplified):
```yaml
robot:
  name: "protectron"
  ros-version: "humble"

  bricks:
    includes:
      - "hardware.yaml"

    state-publisher:
      id: "state-publisher"
      host: "main"
      package:
        build: "yes"
        source:
          local: "state-publisher"
        mounts:
          - "./urdf:state-publisher/description"
        launch: "rsp.launch.py"
```
This snippet references a separate file, `hardware.yaml`, which adds more bricks (e.g., sensors).

```yaml
bricks:

  lidar:
    id: "rplidar"
    host: "main"
    endpoints:
      - topic: "/scan"
        message: "sensor_msgs/LaserScan"
        frame_id: "lidar_link"
        type: "publisher"
    package:
      build: "yes"
      source:
        github: "https://github.com/Slamtec/rplidar_ros"
        branch: "ros2-devel"
      launch: "rplidar_a1.launch"
      params:
        - serial_port: "/dev/ttyUSB0"
        - serial_baudrate: 115200


  microros_agent:
    id: "esp32_microros_agent"
    description: "ESP32 running micro-ROS agent firmware"
    host: "main"
    package:
      build: "yes"
      source:
        github: "https://github.com/micro-ROS/micro-ROS-Agent"
        branch: "humble"
      postinstall:
        - "ros2 run micro_ros_setup create_agent_ws.sh"
        - "ros2 run micro_ros_setup build_agent.sh"
      run: "micro_ros_agent micro_ros_agent"
      params:
        dev: "/dev/ttyACM0"
    endpoints:
      - name: "left_tof_sensor"
        frame_id: "left_tof_link"
        type: "publisher"
        topic: "/left_tof/range"
        message: "sensor_msgs/Range"
      # More endpoints...
```

#### 3. **`hosts.yaml`**

Describes each **host** that will receive Docker images:
```yaml
hosts:
  - name: "main"
    ip: "192.168.122.95"
    user: "cdr"
    password: ""
    arch: "amd64"

  - name: "camera"
    ip: "192.168.122.175"
    user: "cdr"
    password: ""
    arch: "amd64"

overlay_network: "shared-network"
```
Each host’s `name` is referenced by the bricks’ `host` fields in `robot.yaml`. The tool uses `ip`, `user`, and optional `password`/`keyfile` to connect and build Docker images on that host. For this tool to work you will need to expose the docker socket of the host trough tcp. More info about this (here)[].

---


## Internal Overview 

When the tool runs, it does the following (in broad steps):

1. **Load** your configuration (robot definition + hosts):
   - A **main `robot.yaml`** with a `robot:` key, specifying the robot’s name, ROS version, and a set of “bricks” (software components).
   - **Optional includes** referenced inside `robot.yaml` (like `hardware.yaml`) that add more bricks to the configuration.
   - A **`hosts.yaml`** that lists each host’s connection details (IP, username, architecture, etc.).

2. **Parse** each “brick,” which typically has:
   - **`id`** and **`host`**: The brick’s name/ID and which host it belongs to.
   - A **`package`** section describing how to obtain the source:
     - **Local** source (e.g., a folder in `./bricks/<brick_id>/ros_ws`)
     - **GitHub** repo with a branch to clone.
   - **Optional** fields like `launch`, `params`, or `mounts`.
   - **Endpoints**: Zero or one `endpoints` block (which can be a single object or a list of objects) describing publishers/subscribers, their `topic`, `message` type, and an optional `frame_id`.

3. **Create** a **build directory** locally, including subfolders per-host and per-brick.
4. **Fetch** the source code for each brick:
   - If **GitHub**, it clones the repository (optionally pulling updates if `--pull` is provided).
   - If **local**, it copies from the local folder in `./bricks/<brick_id>/ros_ws`.
5. **Detect** ROS dependencies from `package.xml` files to choose an appropriate base Docker image (e.g., if you have perception or simulation-related packages, it picks a more feature-rich base image).
6. **Generate** a Dockerfile by combining various Dockerfile “segments” (e.g., `base.Dockerfile`, `preinstall.Dockerfile`, etc.). This injects any **preinstall** or **postinstall** commands defined in each brick’s package.
7. **Build** the Docker image **on the remote host** using the `hosts.yaml` connection info. The tool supports building via the Docker Python SDK or optionally running SSH commands if needed.
8. Repeat for all bricks, so each host ends up with exactly the images it needs.
---