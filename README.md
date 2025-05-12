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
`build/<component>/ros_ws/install``

### 3. Deploy: deploy.py
Rsyncs build/<component>/ros_ws/install to each robot.
Runs docker-compose up on each robot.


## Notes & Tips
- RSync optimizations (-az --inplace --no-whole-file --delete) ensure only changed files (and only changed blocks) are sent.

Happy coding & safe deployments!
