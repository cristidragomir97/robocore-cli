# ROS 2 Docker Workflow Template

A **lean, three-step** containerized workflow for developing & deploying ROS 2 packages on robots. 


## Overview

1. **prep.py**  
   - Scans `components/<name>/ros_ws`  
   - Renders `components/<name>/Dockerfile` from `templates/Dockerfile.j2`  
   - Generates a top-level `docker-compose.yml`

2. **build.py**  
   - Spins up a temporary ROS 2 container  
   - Mounts each component’s `ros_ws/src` → `/ros_ws/src/<component>`  
   - Runs `colcon build --symlink-install` into `components/<name>/install`

3. **deploy.py**  
   - If `deploy_mode: image`: `docker-compose up -d` (pulls/runs images)  
   - If `deploy_mode: live`: mounts `components/<name>/install` via SSHFS and runs `docker-compose up -d`

## Getting Started

1. **Clone** this repo.  
2. **Edit** `config.yaml` (see below).  
3. **Add** your components under `components/<component>/ros_ws/src/...`.  

4. Run in order:
   ```bash
   ./scripts/prep.py
   ./scripts/build.py
   ./scripts/deploy.py
