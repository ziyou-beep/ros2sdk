#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  start_backend.sh hand [MODEL] [SIDE] [INTERFACE_NAME] [INTERFACE_TYPE]
  start_backend.sh arm [MODEL] [SIDE] [INTERFACE_NAME] [INTERFACE_TYPE] [WORLD_FRAME]
  start_backend.sh dual-hand [MODEL] [LEFT_INTERFACE] [RIGHT_INTERFACE] [INTERFACE_TYPE]

Examples:
  ./examples/start_backend.sh hand L6 left can0
  ./examples/start_backend.sh arm A7lite left can0
  ./examples/start_backend.sh arm P7 left 192.168.10.21 lbot
  ./examples/start_backend.sh dual-hand L6 can0 can1

Defaults:
  hand: MODEL=L6 SIDE=left INTERFACE_NAME=can0 INTERFACE_TYPE=socketcan
  arm:  MODEL=A7lite SIDE=left INTERFACE_NAME=can0 INTERFACE_TYPE=socketcan WORLD_FRAME=urdf
        MODEL=P7 defaults to INTERFACE_NAME=192.168.10.21 INTERFACE_TYPE=lbot
USAGE
}

normalize_model() {
  printf '%s' "$1" | tr '[:lower:]' '[:upper:]' | tr -d '_-'
}

default_arm_interface_name() {
  case "$(normalize_model "$1")" in
    P7) printf '192.168.10.21' ;;
    *) printf 'can0' ;;
  esac
}

default_arm_interface_type() {
  case "$(normalize_model "$1")" in
    P7) printf 'lbot' ;;
    *) printf 'socketcan' ;;
  esac
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

backend="${1:-hand}"
case "$backend" in
  hand)
    model="${2:-L6}"
    side="${3:-left}"
    interface_name="${4:-can0}"
    interface_type="${5:-socketcan}"
    exec ros2 launch realhand_ros2 hand.launch.py \
      "model:=${model}" \
      "side:=${side}" \
      "interface_name:=${interface_name}" \
      "interface_type:=${interface_type}"
    ;;
  arm)
    model="${2:-A7lite}"
    side="${3:-left}"
    interface_name="${4:-$(default_arm_interface_name "${model}")}"
    interface_type="${5:-$(default_arm_interface_type "${model}")}"
    world_frame="${6:-urdf}"
    exec ros2 launch realhand_ros2 arm.launch.py \
      "model:=${model}" \
      "side:=${side}" \
      "interface_name:=${interface_name}" \
      "interface_type:=${interface_type}" \
      "world_frame:=${world_frame}"
    ;;
  dual-hand)
    model="${2:-L6}"
    left_interface="${3:-can0}"
    right_interface="${4:-can1}"
    interface_type="${5:-socketcan}"
    exec ros2 launch realhand_ros2 dual_hand.launch.py \
      "model:=${model}" \
      "left_interface:=${left_interface}" \
      "right_interface:=${right_interface}" \
      "interface_type:=${interface_type}"
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
