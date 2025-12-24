FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
  python3-pip \
  python3-setuptools \
  python3-venv \
  sqlite3 \
  git \
  ca-certificates \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip

WORKDIR /workspace

COPY pyproject.toml ./

RUN pip install -e .

COPY . /workspace

RUN pip install -e .

ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=0
ENV ROS_DISTRO=humble

CMD ["bash"]
