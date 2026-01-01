#!/usr/bin/env bash
set -e

if command -v git >/dev/null 2>&1; then
  git config --global --add safe.directory /workspace || true
fi

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

exec "$@"
