# docker/ros2_sim/Dockerfile

FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1) Add OSRF key & repo for Gazebo Fortress
RUN apt-get update && apt-get install -y wget gnupg2 lsb-release \
 && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - \
 && echo "deb [arch=$(dpkg --print-architecture)] http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# 2) Install Fortress + ROS-Gazebo bridges & control + other deps
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-ros-ign \
    ros-humble-gazebo-ros-pkgs \  
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-bridge \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-moveit \
    ros-humble-ament-cmake \
    curl git python3-pip python3-colcon-common-extensions \
    libopencv-dev python3-opencv \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# 3) Non-root user for DevContainer
RUN useradd -ms /bin/bash dev
USER dev

WORKDIR /ros2_ws

# 4) Source ROS in every shell
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/dev/.bashrc
