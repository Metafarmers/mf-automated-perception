#!/usr/bin/env bash
set -e

# =====================================
# Config
# =====================================
NUM_JOBS=$(nproc)
PREFIX=/usr/local
ROS_DISTRO=humble

# action: build | clean
ACTION=${1:-build}

# =====================================
# Environment
# =====================================
export DEBIAN_FRONTEND=noninteractive
export CMAKE_PREFIX_PATH=${PREFIX}:${CMAKE_PREFIX_PATH}
export QT_X11_NO_MITSHM=1

if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# =====================================
# Clean mode
# =====================================
if [ "${ACTION}" = "clean" ]; then
  echo "=== CLEAN MODE ==="

  for pkg in gtsam iridescence gtsam_points; do
    if [ -d ${pkg}/build ]; then
      echo "[clean] ${pkg}"
      cd ${pkg}/build
      make uninstall || true
      cd ../..
    fi
  done

  if [ -d ~/ros2_ws ]; then
    echo "[clean] ros2_ws"
    cd ~/ros2_ws
    rm -rf build install log
  fi

  echo "Clean finished"
  exit 0
fi

# =====================================
# Dependencies
# =====================================
apt-get update
apt-get install -y \
  build-essential \
  cmake \
  git \
  libomp-dev \
  libboost-all-dev \
  libmetis-dev \
  libfmt-dev \
  libspdlog-dev \
  libeigen3-dev \
  libglm-dev \
  libglfw3-dev \
  libpng-dev \
  libjpeg-dev

# =====================================
# GTSAM
# =====================================
if [ ! -d gtsam ]; then
  git clone https://github.com/borglab/gtsam
fi

cd gtsam
git checkout 4.3a0
mkdir -p build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=${PREFIX} \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_ENABLE_BOOST_SERIALIZATION=ON

make -j${NUM_JOBS}
make install
cd ../..

# =====================================
# Iridescence (viewer)
# =====================================
if [ ! -d iridescence ]; then
  git clone https://github.com/koide3/iridescence --recursive
fi

cd iridescence
mkdir -p build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=${PREFIX} \
  -DCMAKE_BUILD_TYPE=Release \
  -DIRIDESCENCE_BUILD_EXAMPLES=OFF

make -j${NUM_JOBS}
make install
cd ../..

# =====================================
# gtsam_points (CUDA)
# =====================================
if [ ! -d gtsam_points ]; then
  git clone https://github.com/koide3/gtsam_points
fi

cd gtsam_points
mkdir -p build && cd build

cmake .. \
  -DCMAKE_INSTALL_PREFIX=${PREFIX} \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_WITH_CUDA=ON

make -j${NUM_JOBS}
make install
cd ../..

ldconfig

# =====================================
# GLIM + GLIM ROS2
# =====================================
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone --recurse-submodules https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

echo "[glim_ros2] applying custom glim_ros2 overlay"
rm -rf glim_ros2
cp -r /root/tmp/glim_ros2 glim_ros2
rm -rf /root/tmp/glim_ros2

cd ~/ros2_ws

source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release

# =====================================
# Cleanup source directories after build
# =====================================
echo "[cleanup] removing build source directories to reduce image size"

cd /root && rm -rf \
  gtsam \
  iridescence \
  gtsam_points

echo "[cleanup] source directories removed"