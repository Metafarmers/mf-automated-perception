FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-setuptools \
  python3-venv \
  sqlite3 \
  git \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*

RUN apt update
RUN apt install -y \
  ros-humble-rosbag2-py \
  ros-humble-rclpy \
  ros-humble-sensor-msgs \
  && rm -rf /var/lib/apt/lists/*



RUN python3 -m pip install --upgrade pip

WORKDIR /workspace

COPY . /workspace

RUN pip install -e .

ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=0
ENV ROS_DISTRO=humble

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
