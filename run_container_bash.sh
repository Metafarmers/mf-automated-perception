#!/usr/bin/env bash
set -e

MODE="ro"
IMAGE="mf-eye-runtime:humble"

for arg in "$@"; do
  [ "$arg" = "--dev" ] && MODE="rw" || {
    echo "unknown argument: $arg"
    exit 1
  }
done

docker run -it --rm \
  -v "$(pwd)":/workspace:${MODE} \
  -v "$(realpath data)":/data \
  -v /home/tw/mf/project_manage_vcs/spagri/perception/data/mantis-eye/dongtan_automated_perception/dongtan_mot_routine:/external_data \
  --workdir /workspace \
  ${IMAGE}

