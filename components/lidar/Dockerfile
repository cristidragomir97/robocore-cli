# templates/Dockerfile.j2

# Stage 1: Install dependencies via rosdep
FROM dragomirxyz/robocore_humble AS deps

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/ros_ws

WORKDIR /ros_ws/src
COPY ros_ws/src/ ./

RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    udev \
    cmake \
    build-essential && \
    rm -rf /var/lib/apt/lists/* && \
    bash -lc "\
      source /opt/ros/${ROS_DISTRO}/setup.bash && \
      rosdep update && \
      rosdep install --from-paths src --ignore-src -r -y\
    "

# Stage 2: runtime image
FROM dragomirxyz/robocore_humble

ENV ROS_DISTRO=humble
ENV ROS_WS=/ros_ws

COPY --from=deps /opt/ros/${ROS_DISTRO} /opt/ros/${ROS_DISTRO}
RUN mkdir -p ${ROS_WS}/install && mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

ENTRYPOINT ["/bin/bash","-lc"]
CMD [""]