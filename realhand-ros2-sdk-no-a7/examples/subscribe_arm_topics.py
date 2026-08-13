#!/usr/bin/env python3
"""Subscribe to RealHand arm ROS2 topics and print received data."""

from __future__ import annotations

import argparse
import json
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ArmTopicSubscriber(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_arm_subscriber")
        prefix = f"/realhand/{side}/arm"
        self.create_subscription(JointState, f"{prefix}/joint_state", self.on_state, 10)
        self.create_subscription(String, f"{prefix}/pose", self.on_pose, 10)
        self.get_logger().info(f"Subscribed to arm topics under {prefix}")

    def on_state(self, msg: JointState) -> None:
        payload = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        print_json("joint_state", payload)

    def on_pose(self, msg: String) -> None:
        print_json("pose", parse_json(msg.data))


def parse_json(data: str) -> Any:
    try:
        return json.loads(data)
    except json.JSONDecodeError:
        return data


def print_json(label: str, payload: Any) -> None:
    print(f"\n[{label}]")
    print(json.dumps(payload, indent=2))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="left", choices=["left", "right"])
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ArmTopicSubscriber(args.side)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
