#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
SDK_SETUP="${SCRIPT_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS setup file not found: ${ROS_SETUP}" >&2
    echo "Set ROS_DISTRO if you are not using humble." >&2
    exit 1
fi

if [[ ! -f "${SDK_SETUP}" ]]; then
    echo "SDK setup file not found: ${SDK_SETUP}" >&2
    echo "Build the SDK first:" >&2
    echo "  cd \"${SCRIPT_DIR}\"" >&2
    echo "  source \"${ROS_SETUP}\"" >&2
    echo "  colcon build --symlink-install" >&2
    exit 1
fi

set +u
source "${ROS_SETUP}"
source "${SDK_SETUP}"
set -u

exec ros2 launch realhand_ros2 gui.launch.py "$@"
