#!/usr/bin/env python3
"""Set RealHand arm velocity and acceleration values over ROS2 topics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String

try:
    from realhand_ros2.arm.model import get_arm_model_spec
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from realhand_ros2.arm.model import get_arm_model_spec


class ArmMotionSettingsPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_arm_motion_settings")
        self.topic = f"/realhand/{side}/arm/command_json"
        self.publisher = self.create_publisher(String, self.topic, 10)

    def publish_payload(self, payload: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)


def parse_csv_floats(value: str | None) -> list[float] | None:
    if value is None:
        return None
    values = [item.strip() for item in value.split(",")]
    if not values or any(item == "" for item in values):
        raise argparse.ArgumentTypeError("expected comma-separated numbers")
    return [float(item) for item in values]


def validate_count(label: str, values: list[float] | None, expected: int) -> None:
    if values is not None and len(values) != expected:
        raise SystemExit(f"{label} needs {expected} values, got {len(values)}")


def validate_range(
    label: str, values: list[float] | None, value_range: tuple[float, float]
) -> None:
    if values is None:
        return
    low, high = value_range
    for index, value in enumerate(values, start=1):
        if not (low <= value <= high):
            raise SystemExit(
                f"{label} value {index} must be in [{low}, {high}], got {value}"
            )


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
    parser.add_argument("--model", default="A7lite")
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument(
        "--velocities",
        type=parse_csv_floats,
        help="Seven joint velocity limits/targets.",
    )
    parser.add_argument(
        "--accelerations",
        type=parse_csv_floats,
        help="Seven joint accelerations.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait for the arm backend subscriber before publishing.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.velocities is None and args.accelerations is None:
        raise SystemExit("provide --velocities, --accelerations, or both")

    spec = get_arm_model_spec(args.model)
    validate_count("--velocities", args.velocities, spec.joint_count)
    validate_count("--accelerations", args.accelerations, spec.joint_count)
    validate_range("--velocities", args.velocities, spec.velocity_range)
    validate_range("--accelerations", args.accelerations, spec.acceleration_range)

    payload: dict[str, object] = {}
    if args.velocities is not None:
        payload["velocities"] = args.velocities
    if args.accelerations is not None:
        payload["accelerations"] = args.accelerations

    rclpy.init()
    node = ArmMotionSettingsPublisher(args.side)
    try:
        if not wait_for_subscribers(node, node.publisher, args.wait_timeout):
            raise SystemExit(f"no subscriber found on {node.topic}")
        node.publish_payload(payload)
        node.get_logger().info(f"Published motion settings to {node.topic}: {payload}")
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
