#!/usr/bin/env python3
"""Publish example RealHand hand commands over ROS2 topics."""

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
    from realhand_ros2.hand.model import get_hand_model_spec
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from realhand_ros2.hand.model import get_hand_model_spec


class HandCommandPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_hand_command_publisher")
        prefix = f"/realhand/{side}/hand"
        self.command_pub = self.create_publisher(JointState, f"{prefix}/command", 10)
        self.json_pub = self.create_publisher(String, f"{prefix}/command_json", 10)

    def publish_joint_state(
        self,
        names: list[str],
        positions: list[float] | None,
        velocities: list[float] | None,
        efforts: list[float] | None,
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions or []
        msg.velocity = velocities or []
        msg.effort = efforts or []
        self.command_pub.publish(msg)

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
    parser.add_argument("--model", default="L6")
    parser.add_argument(
        "--positions",
        type=parse_csv_floats,
        help="Comma-separated hand positions in the 0..100 range.",
    )
    parser.add_argument(
        "--velocities",
        type=parse_csv_floats,
        help="Comma-separated hand speeds in the 0..100 range.",
    )
    parser.add_argument(
        "--torques",
        type=parse_csv_floats,
        help="Comma-separated hand torques in the 0..100 range.",
    )
    parser.add_argument("--open", action="store_true", help="Send all positions as 100.")
    parser.add_argument("--close", action="store_true", help="Send all positions as 0.")
    parser.add_argument(
        "--json",
        help="Publish raw JSON on command_json. Use this by itself.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    spec = get_hand_model_spec(args.model)
    joint_count = len(spec.joint_names)

    positions = args.positions
    if args.open and args.close:
        raise SystemExit("choose only one of --open or --close")
    if args.open:
        positions = [100.0] * joint_count
    if args.close:
        positions = [0.0] * joint_count

    validate_count("--positions", positions, joint_count)
    validate_count("--velocities", args.velocities, joint_count)
    validate_count("--torques", args.torques, joint_count)

    typed_command_count = sum(
        bool(value) for value in [positions, args.velocities, args.torques]
    )
    if args.json is None and typed_command_count == 0:
        raise SystemExit(
            "provide --positions, --velocities, --torques, --open, --close, or --json"
        )
    if args.json is not None and typed_command_count > 0:
        raise SystemExit("--json must be used by itself")

    rclpy.init()
    node = HandCommandPublisher(args.side)
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
            node.get_logger().info(f"Published JSON command to {args.side} hand")
        else:
            node.publish_joint_state(
                list(spec.joint_names), positions, args.velocities, args.torques
            )
            node.get_logger().info(f"Published JointState command to {args.side} hand")

        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
