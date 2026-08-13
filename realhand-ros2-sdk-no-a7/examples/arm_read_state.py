#!/usr/bin/env python3
"""Read one RealHand arm joint_state and pose sample from ROS2 topics."""

from __future__ import annotations

import argparse
import json
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ArmStateReader(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_arm_read_state")
        prefix = f"/realhand/{side}/arm"
        self.joint_state_topic = f"{prefix}/joint_state"
        self.pose_topic = f"{prefix}/pose"
        self.joint_state: dict[str, object] | None = None
        self.pose: Any | None = None
        self.create_subscription(
            JointState, self.joint_state_topic, self._on_joint_state, 10
        )
        self.create_subscription(String, self.pose_topic, self._on_pose, 10)

    def _on_joint_state(self, msg: JointState) -> None:
        self.joint_state = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }

    def _on_pose(self, msg: String) -> None:
        try:
            self.pose = json.loads(msg.data)
        except json.JSONDecodeError:
            self.pose = msg.data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--state-only", action="store_true")
    parser.add_argument("--pose-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.state_only and args.pose_only:
        raise SystemExit("choose only one of --state-only or --pose-only")

    want_state = not args.pose_only
    want_pose = not args.state_only

    rclpy.init()
    node = ArmStateReader(args.side)
    try:
        deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1_000_000_000)
        while node.get_clock().now().nanoseconds < deadline:
            state_ready = not want_state or node.joint_state is not None
            pose_ready = not want_pose or node.pose is not None
            if state_ready and pose_ready:
                break
            rclpy.spin_once(node, timeout_sec=0.05)

        output: dict[str, object] = {}
        if want_state and node.joint_state is not None:
            output["joint_state"] = node.joint_state
        if want_pose and node.pose is not None:
            output["pose"] = node.pose

        if (want_state and node.joint_state is None) or (want_pose and node.pose is None):
            missing = []
            if want_state and node.joint_state is None:
                missing.append(node.joint_state_topic)
            if want_pose and node.pose is None:
                missing.append(node.pose_topic)
            raise SystemExit(f"timed out waiting for: {', '.join(missing)}")

        print(json.dumps(output, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
