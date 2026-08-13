#!/usr/bin/env python3
"""Publish example RealHand arm commands over ROS2 topics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from realhand_ros2.arm.model import get_arm_model_spec
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from realhand_ros2.arm.model import get_arm_model_spec


class ArmCommandPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_arm_command_publisher")
        prefix = f"/realhand/{side}/arm"
        self.joint_pub = self.create_publisher(JointState, f"{prefix}/joint_command", 10)
        self.json_pub = self.create_publisher(String, f"{prefix}/command_json", 10)

    def publish_joint_state(
        self,
        names: list[str],
        positions: list[float] | None,
        velocities: list[float] | None,
        accelerations: list[float] | None,
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions or []
        msg.velocity = velocities or []
        msg.effort = accelerations or []
        self.joint_pub.publish(msg)

    def publish_json(self, payload: dict[str, object]) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.json_pub.publish(msg)


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument("--model", default="A7lite")
    parser.add_argument("--joints", type=parse_csv_floats, help="Seven joint angles in radians.")
    parser.add_argument("--velocities", type=parse_csv_floats, help="Seven joint velocities.")
    parser.add_argument(
        "--accelerations",
        type=parse_csv_floats,
        help="Seven joint accelerations. Published in JointState.effort.",
    )
    parser.add_argument(
        "--action",
        choices=[
            "enable",
            "disable",
            "home",
            "reset_error",
            "emergency_stop",
            "resume_from_emergency_stop",
        ],
        help="Publish a JSON arm action.",
    )
    parser.add_argument(
        "--json",
        help="Publish raw JSON on command_json. Use this by itself.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    spec = get_arm_model_spec(args.model)
    joint_count = len(spec.joint_names)

    validate_count("--joints", args.joints, joint_count)
    validate_count("--velocities", args.velocities, joint_count)
    validate_count("--accelerations", args.accelerations, joint_count)

    command_count = sum(
        bool(value)
        for value in [
            args.joints,
            args.velocities,
            args.accelerations,
            args.action,
            args.json,
        ]
    )
    if command_count == 0:
        raise SystemExit("provide --joints, --velocities, --accelerations, --action, or --json")
    if args.json is not None and command_count > 1:
        raise SystemExit("--json must be used by itself")

    rclpy.init()
    node = ArmCommandPublisher(args.side)
    try:
        deadline = node.get_clock().now().nanoseconds + 500_000_000
        while node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        if args.json is not None:
            try:
                payload = json.loads(args.json)
            except json.JSONDecodeError as exc:
                raise SystemExit(f"invalid --json payload: {exc}") from exc
            node.publish_json(payload)
            node.get_logger().info(f"Published JSON command to {args.side} arm")
        elif args.action is not None:
            node.publish_json({"action": args.action})
            node.get_logger().info(f"Published {args.action} action to {args.side} arm")
        else:
            node.publish_joint_state(
                list(spec.joint_names), args.joints, args.velocities, args.accelerations
            )
            node.get_logger().info(f"Published JointState command to {args.side} arm")

        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
