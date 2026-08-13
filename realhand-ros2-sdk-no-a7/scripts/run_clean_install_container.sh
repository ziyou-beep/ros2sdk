#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-}"
IMAGE="${IMAGE:-}"
CONTAINER_NAME="${CONTAINER_NAME:-}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/workspaces/realhand_ros2_ws}"
LOCAL_SDK_HOST_PATH="${LOCAL_SDK_HOST_PATH:-}"
LOCAL_SDK_CONTAINER_PATH="${LOCAL_SDK_CONTAINER_PATH:-/host/realbot-python-sdk}"
TZ="${TZ:-America/Los_Angeles}"

if [[ -z "${ROS_DISTRO}" ]]; then
    printf 'error: ROS_DISTRO must be set, for example humble or jazzy.\n' >&2
    exit 1
fi

if [[ -z "${IMAGE}" ]]; then
    case "${ROS_DISTRO}" in
        humble)
            IMAGE="ubuntu:22.04"
            ;;
        jazzy)
            IMAGE="ubuntu:24.04"
            ;;
        *)
            printf 'error: unsupported ROS_DISTRO %s. Set IMAGE explicitly.\n' "${ROS_DISTRO}" >&2
            exit 1
            ;;
    esac
fi

if [[ -z "${CONTAINER_NAME}" ]]; then
    CONTAINER_NAME="realhand-ros2-${ROS_DISTRO}-clean-install"
fi

command -v docker >/dev/null 2>&1 || {
    printf 'error: docker is not installed or not on PATH.\n' >&2
    exit 1
}

docker_args=(
    -it
    --name "${CONTAINER_NAME}"
    --hostname "${CONTAINER_NAME}"
    -e ROS_DISTRO="${ROS_DISTRO}"
    -e TZ="${TZ}"
    -e DEBIAN_FRONTEND=noninteractive
    -e WORKSPACE_DIR="${WORKSPACE_DIR}"
    -e LOCAL_SDK_CONTAINER_PATH="${LOCAL_SDK_CONTAINER_PATH}"
    -v "${REPO_DIR}":/host/realhand-ros2-sdk:ro
)

if [[ -n "${LOCAL_SDK_HOST_PATH}" ]]; then
    if [[ ! -d "${LOCAL_SDK_HOST_PATH}" ]]; then
        printf 'error: LOCAL_SDK_HOST_PATH does not exist: %s\n' "${LOCAL_SDK_HOST_PATH}" >&2
        exit 1
    fi
    docker_args+=(-v "${LOCAL_SDK_HOST_PATH}:${LOCAL_SDK_CONTAINER_PATH}:ro")
fi

docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

exec docker run \
    "${docker_args[@]}" \
    "${IMAGE}" \
    bash -lc 'cd /host/realhand-ros2-sdk && exec bash'
