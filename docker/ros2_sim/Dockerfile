# docker/ros2_sim/Dockerfile  ── dev-friendly variant
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

# ──────────────────────────────────────────────
# 1) Extra apt repos + packages you KNOW you’ll need
#    (Gazebo, ros2_control, colcon, etc.)
# ──────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
      wget gnupg2 lsb-release \
  && wget -qO - https://packages.osrfoundation.org/gazebo.key | apt-key add - \
  && echo "deb [arch=$(dpkg --print-architecture)] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable \
         $(lsb_release -cs) main" \
     > /etc/apt/sources.list.d/gazebo-stable.list \
  && apt-get update && apt-get install -y \
      gazebo \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-controller-manager \
      ros-humble-joint-state-publisher-gui \
      ros-humble-xacro \
      ros-humble-moveit \
      ros-humble-ament-cmake \
      python3-pip python3-colcon-common-extensions \
      libopencv-dev python3-opencv \
      curl git \
      libx11-xcb1 \
      libxcb-render0 \
      libxcb-shm0 \
      libxcb-xinerama0 \
      libxkbcommon-x11-0 \
      libgl1-mesa-glx \
      libglu1-mesa \
  && apt-get clean && rm -rf /var/lib/apt/lists/*

# ──────────────────────────────────────────────
# 2) Non-root dev user
# ──────────────────────────────────────────────
RUN useradd -ms /bin/bash dev

# ──────────────────────────────────────────────
# 3) Workspace skeleton only – NO BUILD
#    (Rebuild quickly inside the running container)
# ──────────────────────────────────────────────
USER root
WORKDIR /ros2_ws
COPY ./ros2_ws /ros2_ws

# ──────────────────────────────────────────────
# 4) Convenience: ROS setup in every shell
#    (overlay sourcing left to you after each build)
# ──────────────────────────────────────────────
USER dev
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/dev/.bashrc

WORKDIR /ros2_ws
CMD ["/bin/bash"]
