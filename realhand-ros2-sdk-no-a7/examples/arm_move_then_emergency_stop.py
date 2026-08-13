#!/usr/bin/env python3
"""Send one arm motion command, then publish emergency_stop after a delay."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from realhand_ros2.arm.model import get_arm_model_spec
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from realhand_ros2.arm.model import get_arm_model_spec


DEFAULT_POSITIONS = [-0.2, 0.1, 0.2, -0.2, 0.0, 0.0, 0.0]
DEFAULT_VELOCITIES = [0.1] * 7
DEFAULT_ACCELERATIONS = [1.0] * 7


class ArmMoveThenStopPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("realhand_example_arm_move_then_emergency_stop")
        prefix = f"/realhand/{side}/arm"
        self.joint_topic = f"{prefix}/joint_command"
        self.json_topic = f"{prefix}/command_json"
        self.joint_pub = self.create_publisher(JointState, self.joint_topic, 10)
        self.json_pub = self.create_publisher(String, self.json_topic, 10)

    def publish_joint_state(
        self,
        names: list[str],
        positions: list[float],
        velocities: list[float],
        accelerations: list[float],
    ) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        msg.velocity = velocities
        msg.effort = accelerations
        self.joint_pub.publish(msg)

    def publish_emergency_stop(self) -> None:
        msg = String()
        msg.data = json.dumps({"action": "emergency_stop"})
        self.json_pub.publish(msg)


def parse_csv_floats(value: str) -> list[float]:
    values = [item.strip() for item in value.split(",")]
    if not values or any(item == "" for item in values):
        raise argparse.ArgumentTypeError("expected comma-separated numbers")
    return [float(item) for item in values]


def validate_count(label: str, values: list[float], expected: int) -> None:
    if len(values) != expected:
        raise SystemExit(f"{label} needs {expected} values, got {len(values)}")


def validate_range(label: str, values: list[float], value_range: tuple[float, float]) -> None:
    low, high = value_range
    for index, value in enumerate(values, start=1):
        if not (low <= value <= high):
            raise SystemExit(f"{label} value {index} must be in [{low}, {high}], got {value}")


def wait_for_subscribers(node: Node, publisher: Publisher, timeout_s: float) -> bool:
    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1_000_000_000)
    while node.get_clock().now().nanoseconds < deadline:
        if publisher.get_subscription_count() > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return publisher.get_subscription_count() > 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="P7")
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument(
        "--positions",
        type=parse_csv_floats,
        default=DEFAULT_POSITIONS,
        help="Seven target joint angles in radians.",
    )
    parser.add_argument(
        "--velocities",
        type=parse_csv_floats,
        default=DEFAULT_VELOCITIES,
        help="Seven joint velocities.",
    )
    parser.add_argument(
        "--accelerations",
        type=parse_csv_floats,
        default=DEFAULT_ACCELERATIONS,
        help="Seven joint accelerations. Published in JointState.effort.",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.5,
        help="Seconds to wait after the move command before emergency_stop.",
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
    spec = get_arm_model_spec(args.model)

    validate_count("--positions", args.positions, spec.joint_count)
    validate_count("--velocities", args.velocities, spec.joint_count)
    validate_count("--accelerations", args.accelerations, spec.joint_count)
    validate_range("--velocities", args.velocities, spec.velocity_range)
    validate_range("--accelerations", args.accelerations, spec.acceleration_range)

    rclpy.init()
    node = ArmMoveThenStopPublisher(args.side)
    try:
        if not wait_for_subscribers(node, node.joint_pub, args.wait_timeout):
            raise SystemExit(f"no subscriber found on {node.joint_topic}")
        if not wait_for_subscribers(node, node.json_pub, args.wait_timeout):
            raise SystemExit(f"no subscriber found on {node.json_topic}")

        node.publish_joint_state(
            list(spec.joint_names), args.positions, args.velocities, args.accelerations
        )
        node.get_logger().info(f"Published JointState command to {node.joint_topic}")
        rclpy.spin_once(node, timeout_sec=0.1)

        if args.delay > 0.0:
            time.sleep(args.delay)

        node.publish_emergency_stop()
        node.get_logger().warning(f"Published emergency_stop to {node.json_topic}")
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
