#!/usr/bin/env python3
"""Send emergency_stop to a RealHand arm over ROS2 topics."""

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String


class ArmEmergencyStopPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_arm_emergency_stop")
        self.topic = f"/realhand/{side}/arm/command_json"
        self.publisher = self.create_publisher(String, self.topic, 10)

    def publish_stop(self) -> None:
        msg = String()
        msg.data = json.dumps({"action": "emergency_stop"})
        self.publisher.publish(msg)


def wait_for_subscribers(
    node: Node, publisher: Publisher, timeout_s: float
) -> bool:
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1_000_000_000)
    while node.get_clock().now().nanoseconds < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return publisher.get_subscription_count() > 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for the arm backend subscriber before publishing.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ArmEmergencyStopPublisher(args.side)
    try:
        if not wait_for_subscribers(node, node.publisher, args.wait_timeout):
            raise SystemExit(f"no subscriber found on {node.topic}")
        node.publish_stop()
        node.get_logger().warning(f"Published emergency_stop to {node.topic}")
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
