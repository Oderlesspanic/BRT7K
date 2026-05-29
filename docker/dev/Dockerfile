FROM --platform=linux/arm64 osrf/ros:jazzy-desktop

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
    ros-dev-tools \
    ros-jazzy-xacro \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-robot-localization \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-camera-ros \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-opencv \
    ros-jazzy-image-transport \
    ros-jazzy-tf-transformations \
    ros-jazzy-laser-filters \
    ros-jazzy-message-filters \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-pipeline \
    ros-jazzy-rosbridge-suite \
    libgmock-dev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
