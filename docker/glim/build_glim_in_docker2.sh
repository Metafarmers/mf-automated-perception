#!/usr/bin/env bash
set -e

# =====================================
# GLIM + GLIM ROS2
# =====================================
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone --recurse-submodules https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

echo "[glim_ros2] applying custom glim_ros2 overlay"
cp -r /root/tmp/glim_ros2/* glim_ros2/
rm -rf /root/tmp/glim_ros2

cd ~/ros2_ws

apt update
apt install -y python3-rosdep
rosdep init
rosdep update

/bin/bash -c "source /opt/ros/humble/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y"
/bin/bash -c "source /opt/ros/humble/setup.bash && colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release"

# =====================================
# Cleanup source directories after build
# =====================================
echo "[cleanup] removing build source directories to reduce image size"

cd /root && rm -rf \
  gtsam \
  iridescence \
  gtsam_points

echo "[cleanup] source directories removed"