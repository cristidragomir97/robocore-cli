# robocore-cli 

A **three‐step**, copy-based workflow for building and deploying ROS 2 packages to remote robots, with native-speed local mounts and zero password prompts.


## 📁 Repository Layout
project-root/
├── build/ ← build artifacts (ros_ws/ under each component)
├── components/
│ └── <component>/
│ ├── ros_ws/ ← your source tree (src/) and package.xml
│ └── component.yaml ← per‐component settings
├── scripts/
│ ├── ssh_setup.py ← generate & exchange SSH keys both ways
│ ├── prep.py ← render Dockerfiles, build & push images, generate compose
│ ├── builder.py ← invoke builder images to produce build/<comp>/ros_ws/install
│ └── deploy.py ← rsync build/ros_ws → robot, then docker-compose up
├── templates/
│ ├── Dockerfile.builder.j2
│ ├── Dockerfile.runtime.j2
│ └── docker-compose.j2
├── utils/
│ └── common.py ← load_config(), get_hosts(), render_template()
├── config.yaml ← global settings (registry, build_dir, etc.)
└── README.md


## ⚙️ Scripts
### 1. Prep & Build Images: prep.py
Renders Dockerfile.builder & Dockerfile.runtime for each component.
Builds & pushes builder images (...:builder-<tag>).
Builds & pushes multi-arch runtime images (...:<tag>).
Pulls runtime images on each host.
Generates docker-compose.yml.

### 2. Build ROS 2 Packages: builder.py
Runs each builder image on your dev machine, mounts in your ros_ws/src, and produces
`build/<component>/ros_ws/install``

### 3. Deploy: deploy.py
Rsyncs build/<component>/ros_ws/install to each robot.
Runs docker-compose up on each robot.


## Notes & Tips
- RSync optimizations (-az --inplace --no-whole-file --delete) ensure only changed files (and only changed blocks) are sent.


Happy coding & safe deployments!
