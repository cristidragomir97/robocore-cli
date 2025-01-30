FROM {BASE_IMAGE}

# Basic system packages
RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \ 
    build-essential \ 
    tree \ 
    && rm -rf /var/lib/apt/lists/*
