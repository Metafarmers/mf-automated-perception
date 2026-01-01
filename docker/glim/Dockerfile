# Base image with CUDA on Ubuntu 22.04
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------
# Basic system packages
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
  curl \
  gnupg \
  lsb-release \
  ca-certificates \
  software-properties-common \
  build-essential \
  cmake \
  git \
  && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# ROS 2 Humble repository
# --------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  > /etc/apt/sources.list.d/ros2.list

# --------------------------------------------------
# Install ROS 2 Humble + common build deps
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  libboost-all-dev \
  libglfw3-dev \
  libmetis-dev \
  ros-humble-rosbag2-storage-mcap \
  && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# Optional but commonly needed math / logging libs
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
  libspdlog-dev \
  && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# Source ROS 2 by default
# --------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspace
COPY . /workspace
RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-setuptools \
  python3-venv \
  sqlite3 \
  git \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*

COPY docker/glim/build_glim_in_docker.sh /root/build_glim_in_docker.sh
COPY docker/glim/glim_ros2 /root/tmp/glim_ros2
WORKDIR /root
RUN /bin/bash -c "/root/build_glim_in_docker.sh"

WORKDIR /workspace
RUN python3 -m pip install --upgrade pip
RUN pip install -e .

COPY docker/glim/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# --------------------------------------------------
# Workspace
# --------------------------------------------------
WORKDIR /workspace

CMD ["bash"]
