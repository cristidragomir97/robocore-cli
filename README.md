# robocore-cli 

A **three‐step**, copy-based workflow for building and deploying ROS 2 packages to remote robots, with native-speed local mounts and zero password prompts.

## ⚙️ Scripts
### 1. Prep & Build Images: prep.py
Renders Dockerfile.builder & Dockerfile.runtime for each component.
Builds & pushes builder images (...:builder-<tag>).
Builds & pushes multi-arch runtime images (...:<tag>).
Pulls runtime images on each host.
Generates docker-compose.yml.

### 2. Build ROS 2 Packages: builder.py
Runs each builder image on your dev machine, mounts in your ros_ws/src, and produces
`build/<component>/ros_ws/install`

### 3. Deploy: deploy.py
Rsyncs build/<component>/ros_ws/install to each robot.
Runs docker-compose up on each robot.


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
