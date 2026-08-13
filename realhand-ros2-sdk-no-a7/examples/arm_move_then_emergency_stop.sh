#!/usr/bin/env bash
set -euo pipefail

SIDE="${1:-left}"
DELAY="${2:-0.5}"

JOINT_TOPIC="/realhand/${SIDE}/arm/joint_command"
JSON_TOPIC="/realhand/${SIDE}/arm/command_json"

ros2 topic pub --once "${JOINT_TOPIC}" sensor_msgs/msg/JointState \
"{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'], position: [-0.2, 0.1, 0.2, -0.2, 0.0, 0.0, 0.0], velocity: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], effort: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"

sleep "${DELAY}"

ros2 topic pub --once --wait-matching-subscriptions 1 "${JSON_TOPIC}" std_msgs/msg/String \
"{data: '{\"action\": \"emergency_stop\"}'}"
