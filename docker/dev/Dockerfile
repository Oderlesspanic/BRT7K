FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    nano \
    vim \
    python3-pip \
    iputils-ping \
    curl \
    tmux \
    python3-colcon-common-extensions \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    libgmock-dev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws

CMD ["bash"]