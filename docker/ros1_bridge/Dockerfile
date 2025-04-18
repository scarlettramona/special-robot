# ────────────────────────────
# Hybrid ROS 1 (Noetic) + ROS 2 (Humble) bridge
# ────────────────────────────
FROM osrf/ros:humble-desktop

# 1. Add ROS 1 repository (Ubuntu 20.04 focal)
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" \
      > /etc/apt/sources.list.d/ros1.list \
 && apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc

# 2. Install ROS 1 base + bridge deps
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-noetic-ros-base \
        python3-rosdep \
        python3-colcon-common-extensions \
        ros-humble-ros1-bridge && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# 3. Runtime defaults
ENV ROS_DISTRO=humble
ENTRYPOINT ["ros2", "run", "ros1_bridge", "dynamic_bridge"]
