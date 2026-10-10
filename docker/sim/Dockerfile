FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get -o Acquire::Retries=3 -o Acquire::http::Pipeline-Depth=0 update && \
    apt-get -o Acquire::Retries=3 -o Acquire::http::Pipeline-Depth=0 install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ros-gz \
    ros-humble-joy-linux \
    ros-humble-pcl-ros \
    ros-humble-depth-image-proc \
    ros-humble-cv-bridge \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    git \
    curl \
    wget \
    mesa-utils \
    x11-apps \
    xterm \
    xfonts-base \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true

WORKDIR /workspace

CMD ["/bin/bash"]
