FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-setuptools \
  python3-venv \
  sqlite3 \
  git \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*
  
COPY docker/mf-mantis-eye/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt

# ros 관련 system deps
RUN apt-get update && \
  apt-get install -y \
    ros-humble-rosbag2-py \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-cv-bridge \
    libgl1 \
    libglib2.0-0 \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspace
COPY . /workspace
RUN python3 -m pip install --upgrade pip
RUN pip install -e .


ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=0
ENV ROS_DISTRO=humble

COPY docker/mf-mantis-eye/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
