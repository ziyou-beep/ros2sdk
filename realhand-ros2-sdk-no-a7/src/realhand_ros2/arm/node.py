"""ROS2 node for RealHand Python SDK arm classes."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from realhand_ros2.arm.adapter import ArmAdapter
from realhand_ros2.arm.model import get_arm_model_spec


class ArmNode(Node):
    """Expose a RealHand arm as ROS2 topics."""

    def __init__(self) -> None:
        super().__init__("realhand_arm_node")

        self.declare_parameter("model", "A7lite")
        self.declare_parameter("side", "left")
        self.declare_parameter("interface_name", "")
        self.declare_parameter("interface_type", "")
        self.declare_parameter("world_frame", "urdf")
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("connect_on_start", True)
        self.declare_parameter("state_rate_hz", 50.0)
        self.declare_parameter("joint_command_topic", "")
        self.declare_parameter("json_command_topic", "")
        self.declare_parameter("joint_state_topic", "")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("control_angle_topic", "")
        self.declare_parameter("control_velocity_topic", "")
        self.declare_parameter("control_acceleration_topic", "")
        self.declare_parameter("temperature_topic", "")
        self.declare_parameter("moving_topic", "")
        self.declare_parameter("joint_limits_topic", "")
        self.declare_parameter("state_snapshot_topic", "")

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.model_spec = get_arm_model_spec(self.model)
        self.side = self.get_parameter("side").get_parameter_value().string_value
        self.interface_name = (
            self.get_parameter("interface_name").get_parameter_value().string_value
            or self.model_spec.default_interface_name
        )
        self.interface_type = (
            self.get_parameter("interface_type").get_parameter_value().string_value
            or self.model_spec.default_interface_type
        )
        self.world_frame = (
            self.get_parameter("world_frame").get_parameter_value().string_value
        )
        self.tcp_offset = [float(value) for value in self.get_parameter("tcp_offset").value]
        state_rate_hz = self.get_parameter("state_rate_hz").value

        joint_command_topic = self._topic_param(
            "joint_command_topic", f"/realhand/{self.side}/arm/joint_command"
        )
        json_command_topic = self._topic_param(
            "json_command_topic", f"/realhand/{self.side}/arm/command_json"
        )
        joint_state_topic = self._topic_param(
            "joint_state_topic", f"/realhand/{self.side}/arm/joint_state"
        )
        pose_topic = self._topic_param("pose_topic", f"/realhand/{self.side}/arm/pose")
        control_angle_topic = self._topic_param(
            "control_angle_topic", f"/realhand/{self.side}/arm/control_angle"
        )
        control_velocity_topic = self._topic_param(
            "control_velocity_topic", f"/realhand/{self.side}/arm/control_velocity"
        )
        control_acceleration_topic = self._topic_param(
            "control_acceleration_topic",
            f"/realhand/{self.side}/arm/control_acceleration",
        )
        temperature_topic = self._topic_param(
            "temperature_topic", f"/realhand/{self.side}/arm/temperature"
        )
        moving_topic = self._topic_param(
            "moving_topic", f"/realhand/{self.side}/arm/moving"
        )
        joint_limits_topic = self._topic_param(
            "joint_limits_topic", f"/realhand/{self.side}/arm/joint_limits"
        )
        state_snapshot_topic = self._topic_param(
            "state_snapshot_topic", f"/realhand/{self.side}/arm/state_snapshot"
        )

        self.adapter: ArmAdapter | None = None
        if self.get_parameter("connect_on_start").get_parameter_value().bool_value:
            self.adapter = ArmAdapter(
                model=self.model,
                side=self.side,
                interface_name=self.interface_name,
                interface_type=self.interface_type,
                tcp_offset=self.tcp_offset,
                world_frame=self.world_frame,
            )
            self.get_logger().info(
                f"Connected {self.model} {self.side} arm on {self.interface_name}"
            )
        else:
            self.get_logger().warning("connect_on_start is false; hardware is not open")

        self.joint_state_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self.pose_pub = self.create_publisher(String, pose_topic, 10)
        self.control_angle_pub = self.create_publisher(String, control_angle_topic, 10)
        self.control_velocity_pub = self.create_publisher(
            String, control_velocity_topic, 10
        )
        self.control_acceleration_pub = self.create_publisher(
            String, control_acceleration_topic, 10
        )
        self.temperature_pub = self.create_publisher(String, temperature_topic, 10)
        self.moving_pub = self.create_publisher(String, moving_topic, 10)
        self.joint_limits_pub = self.create_publisher(String, joint_limits_topic, 10)
        self.state_snapshot_pub = self.create_publisher(String, state_snapshot_topic, 10)
        self.create_subscription(
            JointState, joint_command_topic, self._on_joint_command, 10
        )
        self.create_subscription(String, json_command_topic, self._on_json_command, 10)

        period = 1.0 / float(state_rate_hz) if state_rate_hz else 1.0 / 50.0
        self.create_timer(period, self._publish_state)

    def _topic_param(self, name: str, fallback: str) -> str:
        value = self.get_parameter(name).get_parameter_value().string_value
        return value or fallback

    def _on_joint_command(self, msg: JointState) -> None:
        if self.adapter is None:
            self.get_logger().warning("Ignoring arm command; hardware is not connected")
            return
        try:
            if msg.velocity:
                self.adapter.set_velocities(msg.velocity)
            if msg.effort:
                self.adapter.set_accelerations(msg.effort)
            if msg.position:
                self.adapter.move_joints(msg.position, blocking=False)
        except Exception as exc:
            self.get_logger().error(f"Arm command failed: {exc}")

    def _on_json_command(self, msg: String) -> None:
        if self.adapter is None:
            self.get_logger().warning("Ignoring JSON command; hardware is not connected")
            return
        try:
            payload = json.loads(msg.data)
            self._apply_json_command(payload)
        except Exception as exc:
            self.get_logger().error(f"JSON arm command failed: {exc}")

    def _apply_json_command(self, payload: dict[str, Any]) -> None:
        if self.adapter is None:
            return
        if "velocities" in payload:
            self.adapter.set_velocities(payload["velocities"])
        if "accelerations" in payload:
            self.adapter.set_accelerations(payload["accelerations"])

        action = payload.get("action")
        if action == "enable":
            self.adapter.enable()
        elif action == "disable":
            self.adapter.disable()
        elif action == "home":
            self.adapter.home(blocking=bool(payload.get("blocking", False)))
        elif action == "reset_error":
            self.adapter.reset_error()
        elif action == "emergency_stop":
            self.adapter.emergency_stop(bool(payload.get("enable", True)))
        elif action == "resume_from_emergency_stop":
            self.adapter.resume_from_emergency_stop()

        if "joints" in payload:
            self.adapter.move_joints(
                payload["joints"], blocking=bool(payload.get("blocking", False))
            )

    def _publish_state(self) -> None:
        if self.adapter is None:
            return
        try:
            positions, velocities, efforts = self.adapter.joint_state()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.adapter.model.joint_names)
            msg.position = positions
            msg.velocity = velocities
            msg.effort = efforts
            self.joint_state_pub.publish(msg)

            pose_msg = String()
            pose_msg.data = json.dumps(self.adapter.pose_payload())
            self.pose_pub.publish(pose_msg)

            self._publish_optional_list(self.control_angle_pub, self.adapter.control_angles())
            self._publish_optional_list(
                self.control_velocity_pub, self.adapter.control_velocities()
            )
            self._publish_optional_list(
                self.control_acceleration_pub, self.adapter.control_accelerations()
            )
            self._publish_optional_list(self.temperature_pub, self.adapter.temperatures())

            is_moving = self.adapter.is_moving()
            if is_moving is not None:
                self._publish_json(self.moving_pub, {"is_moving": is_moving})

            joint_limits = self.adapter.joint_limits()
            if joint_limits is not None:
                self._publish_json(
                    self.joint_limits_pub,
                    [
                        {"lower": lower, "upper": upper}
                        for lower, upper in joint_limits
                    ],
                )

            self._publish_json(self.state_snapshot_pub, self.adapter.state_payload())
        except Exception as exc:
            self.get_logger().error(f"Failed to publish arm state: {exc}")

    def _publish_json(self, publisher: Any, payload: object) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)

    def _publish_optional_list(self, publisher: Any, values: list[float] | None) -> None:
        if values is None:
            return
        self._publish_json(publisher, values)

    def destroy_node(self) -> bool:
        if self.adapter is not None:
            self.adapter.close()
            self.adapter = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
