#!/usr/bin/env bash
set -e

if command -v git >/dev/null 2>&1; then
  git config --global --add safe.directory /workspace || true
fi

. /opt/ros/humble/setup.bash

exec "$@"
