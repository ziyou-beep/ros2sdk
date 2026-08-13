#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export ROS_DISTRO=humble
export IMAGE="${IMAGE:-ubuntu:22.04}"
export CONTAINER_NAME="${CONTAINER_NAME:-realhand-ros2-humble-clean-install}"

exec "${SCRIPT_DIR}/run_clean_install_container.sh"
