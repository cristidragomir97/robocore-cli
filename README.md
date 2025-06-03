# robocore-cli

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

## Configuration
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
    ports: []
```

- **name**: The unique identifier for this component
- **folder**: Where its ROS 2 workspace lives
- **runs_on**: Which host this component deploys to
- **entrypoint**: What command runs at container start
- **devices**: Which devices to expose (e.g., /dev/ttyUSB0)
- **apt_packages**: List of apt packages you want to install
- **repositories**: Define dependencies declaratively
- **postinstall**: Commands to run after installing dependencies.

---



## Notes & Tips

### Enabling TCP socket for docker hosts.
roscore-cli aims to be as transparent as possible to the systems installed on the robots. 

1. Create `daemon.json` file in `/etc/docker`:

        {"hosts": ["tcp://0.0.0.0:2375", "unix:///var/run/docker.sock"]}

2. Add `/etc/systemd/system/docker.service.d/override.conf`

        [Service]
        ExecStart=
        ExecStart=/usr/bin/dockerd

3. Reload the systemd daemon:

        systemctl daemon-reload

4. Restart docker:

        systemctl restart docker.service

> [!WARNING]  
> 
