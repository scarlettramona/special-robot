FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

# basic tools
RUN apt-get update && apt-get install -y curl git lsb-release

# ROS 2 base + development tools
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y ros-$ROS_DISTRO-desktop python3-colcon-common-extensions

# create user
RUN useradd -ms /bin/bash dev
USER dev
WORKDIR /ros2_ws

# source ROS 2 every shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
