# Initialize rosdep (ignore if already initialized)

# Create a workspace, copy entire build context into /ros2_ws/src

# Install dependencies for *all* packages under src
RUN apt update && ROSDEP_DEBUG=1 rosdep install -i --rosdistro {ROS_DISTRO}  --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# Build all packages discovered in /ros2_ws/src
RUN . /opt/ros/{ROS_DISTRO}/setup.sh && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
