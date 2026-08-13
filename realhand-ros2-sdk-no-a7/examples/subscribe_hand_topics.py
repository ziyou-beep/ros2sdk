#!/usr/bin/env python3
"""Subscribe to RealHand hand ROS2 topics and print received data."""

from __future__ import annotations

import argparse
import json
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HandTopicSubscriber(Node):
    def __init__(self, side: str, all_sensors: bool) -> None:
        super().__init__("realhand_example_hand_subscriber")
        prefix = f"/realhand/{side}/hand"
        self.create_subscription(JointState, f"{prefix}/state", self.on_state, 10)
        self.create_subscription(String, f"{prefix}/snapshot", self.on_snapshot, 10)
        self.create_subscription(String, f"{prefix}/device_info", self.on_device_info, 10)
        self.create_subscription(
            String, f"{prefix}/control_status", self.on_control_status, 10
        )
        self.create_subscription(
            String, f"{prefix}/blocking_result", self.on_blocking_result, 10
        )

        sensor_topics = ["touch", "temperature", "current"]
        if all_sensors:
            sensor_topics.extend(
                [
                    "angle",
                    "speed",
                    "torque",
                    "acceleration",
                    "force_sensor",
                    "fault",
                ]
            )

        for name in sensor_topics:
            self.create_subscription(
                String,
                f"{prefix}/{name}",
                lambda msg, topic=name: self.on_json_topic(topic, msg),
                10,
            )

        self.get_logger().info(f"Subscribed to hand topics under {prefix}")

    def on_state(self, msg: JointState) -> None:
        payload = {
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        print_json("state", payload)

    def on_snapshot(self, msg: String) -> None:
        print_json("snapshot", parse_json(msg.data))

    def on_device_info(self, msg: String) -> None:
        print_json("device_info", parse_json(msg.data))

    def on_control_status(self, msg: String) -> None:
        print_json("control_status", parse_json(msg.data))

    def on_blocking_result(self, msg: String) -> None:
        print_json("blocking_result", parse_json(msg.data))

    def on_json_topic(self, topic: str, msg: String) -> None:
        print_json(topic, parse_json(msg.data))


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
    parser.add_argument(
        "--all-sensors",
        action="store_true",
        help="Also subscribe to angle, speed, torque, acceleration, force, and fault topics.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = HandTopicSubscriber(args.side, args.all_sensors)
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
