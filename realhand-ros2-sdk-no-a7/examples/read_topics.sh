#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  read_topics.sh hand [SIDE] [list|info|echo|echo-once]
  read_topics.sh arm [SIDE] [list|info|echo|echo-once]

Examples:
  ./examples/read_topics.sh hand left list
  ./examples/read_topics.sh hand left info
  ./examples/read_topics.sh hand left echo-once
  ./examples/read_topics.sh arm left echo
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

backend="${1:-hand}"
side="${2:-left}"
mode="${3:-list}"

case "$backend" in
  hand)
    topics=(
      "/realhand/${side}/hand/command"
      "/realhand/${side}/hand/command_json"
      "/realhand/${side}/hand/state"
      "/realhand/${side}/hand/snapshot"
      "/realhand/${side}/hand/device_info"
      "/realhand/${side}/hand/control_status"
      "/realhand/${side}/hand/blocking_result"
      "/realhand/${side}/hand/angle"
      "/realhand/${side}/hand/speed"
      "/realhand/${side}/hand/torque"
      "/realhand/${side}/hand/acceleration"
      "/realhand/${side}/hand/temperature"
      "/realhand/${side}/hand/current"
      "/realhand/${side}/hand/touch"
      "/realhand/${side}/hand/force_sensor"
      "/realhand/${side}/hand/fault"
    )
    echo_topics=(
      "/realhand/${side}/hand/state"
      "/realhand/${side}/hand/snapshot"
      "/realhand/${side}/hand/device_info"
      "/realhand/${side}/hand/touch"
      "/realhand/${side}/hand/temperature"
      "/realhand/${side}/hand/current"
    )
    ;;
  arm)
    topics=(
      "/realhand/${side}/arm/joint_command"
      "/realhand/${side}/arm/command_json"
      "/realhand/${side}/arm/joint_state"
      "/realhand/${side}/arm/pose"
    )
    echo_topics=(
      "/realhand/${side}/arm/joint_state"
      "/realhand/${side}/arm/pose"
    )
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

case "$mode" in
  list)
    ros2 topic list
    ;;
  info)
    for topic in "${topics[@]}"; do
      echo
      echo "== ${topic} =="
      ros2 topic info -v "$topic" || true
    done
    ;;
  echo)
    for topic in "${echo_topics[@]}"; do
      echo "Echoing ${topic}. Press Ctrl-C to stop."
      ros2 topic echo --full-length "$topic"
    done
    ;;
  echo-once)
    for topic in "${echo_topics[@]}"; do
      echo
      echo "== ${topic} =="
      ros2 topic echo --once --full-length "$topic" || true
    done
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
