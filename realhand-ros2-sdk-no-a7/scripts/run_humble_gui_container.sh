#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="${CONTAINER_NAME:-realhand-ros2-humble-gui}"
IMAGE="${IMAGE:-ubuntu:22.04}"
TZ="${TZ:-America/Los_Angeles}"

if [[ -z "${DISPLAY:-}" ]]; then
    printf 'error: DISPLAY is not set. Run this from a graphical host session.\n' >&2
    exit 1
fi

command -v docker >/dev/null 2>&1 || {
    printf 'error: docker is not installed or not on PATH.\n' >&2
    exit 1
}

command -v xhost >/dev/null 2>&1 || {
    printf 'error: xhost is not installed or not on PATH.\n' >&2
    exit 1
}

xhost +local:docker
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

exec docker run -it \
    --name "${CONTAINER_NAME}" \
    --hostname "${CONTAINER_NAME}" \
    --network host \
    --privileged \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e QT_XCB_GL_INTEGRATION=none \
    -e QT_QUICK_BACKEND=software \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e TZ="${TZ}" \
    -e DEBIAN_FRONTEND=noninteractive \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${REPO_DIR}":/host/realhand-ros2-sdk:ro \
    "${IMAGE}" \
    bash -lc 'mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root && cd /host/realhand-ros2-sdk && exec bash'
