#!/usr/bin/env bash
set -e

# --------------------------------------------------
# Git safety (for mounted host repo)
# --------------------------------------------------
if command -v git >/dev/null 2>&1; then
  # Allow git operations on mounted workspace
  git config --global --add safe.directory /workspace || true
fi


source /opt/ros/humble/setup.bash

exec "$@"

