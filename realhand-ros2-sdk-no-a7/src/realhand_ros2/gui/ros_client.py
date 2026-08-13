"""ROS2 client node used by the Qt GUI."""

from __future__ import annotations

from collections.abc import Callable

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class GuiRosClient(Node):
    """Small ROS facade for GUI publishers and subscriptions."""

    def __init__(self) -> None:
        super().__init__("realhand_gui")
        self._hand_command_pub = None
        self._hand_json_pub = None
        self._arm_joint_pub = None
        self._arm_json_pub = None
        self._hand_subscriptions = []
        self._arm_subscriptions = []
        self.configure_hand("left", None, None)
        self.configure_arm("left", None, None)

    def configure_hand(
        self,
        side: str,
        state_callback: Callable[[JointState], None] | None,
        snapshot_callback: Callable[[String], None] | None,
        device_info_callback: Callable[[String], None] | None = None,
        temperature_callback: Callable[[String], None] | None = None,
        current_callback: Callable[[String], None] | None = None,
        touch_callback: Callable[[String], None] | None = None,
        control_status_callback: Callable[[String], None] | None = None,
        blocking_result_callback: Callable[[String], None] | None = None,
    ) -> None:
        if self._hand_command_pub is not None:
            self.destroy_publisher(self._hand_command_pub)
        if self._hand_json_pub is not None:
            self.destroy_publisher(self._hand_json_pub)
        for subscription in self._hand_subscriptions:
            self.destroy_subscription(subscription)
        self._hand_subscriptions = []

        self._hand_command_pub = self.create_publisher(
            JointState, f"/realhand/{side}/hand/command", 10
        )
        self._hand_json_pub = self.create_publisher(
            String, f"/realhand/{side}/hand/command_json", 10
        )
        if state_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    JointState, f"/realhand/{side}/hand/state", state_callback, 10
                )
            )
        if snapshot_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String, f"/realhand/{side}/hand/snapshot", snapshot_callback, 10
                )
            )
        if device_info_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String,
                    f"/realhand/{side}/hand/device_info",
                    device_info_callback,
                    10,
                )
            )
        if temperature_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String,
                    f"/realhand/{side}/hand/temperature",
                    temperature_callback,
                    10,
                )
            )
        if current_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String, f"/realhand/{side}/hand/current", current_callback, 10
                )
            )
        if touch_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String, f"/realhand/{side}/hand/touch", touch_callback, 10
                )
            )
        if control_status_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String,
                    f"/realhand/{side}/hand/control_status",
                    control_status_callback,
                    10,
                )
            )
        if blocking_result_callback is not None:
            self._hand_subscriptions.append(
                self.create_subscription(
                    String,
                    f"/realhand/{side}/hand/blocking_result",
                    blocking_result_callback,
                    10,
                )
            )

    def configure_arm(
        self,
        side: str,
        state_callback: Callable[[JointState], None] | None,
        pose_callback: Callable[[String], None] | None,
    ) -> None:
        if self._arm_joint_pub is not None:
            self.destroy_publisher(self._arm_joint_pub)
        if self._arm_json_pub is not None:
            self.destroy_publisher(self._arm_json_pub)
        for subscription in self._arm_subscriptions:
            self.destroy_subscription(subscription)
        self._arm_subscriptions = []

        self._arm_joint_pub = self.create_publisher(
            JointState, f"/realhand/{side}/arm/joint_command", 10
        )
        self._arm_json_pub = self.create_publisher(
            String, f"/realhand/{side}/arm/command_json", 10
        )
        if state_callback is not None:
            self._arm_subscriptions.append(
                self.create_subscription(
                    JointState, f"/realhand/{side}/arm/joint_state", state_callback, 10
                )
            )
        if pose_callback is not None:
            self._arm_subscriptions.append(
                self.create_subscription(
                    String, f"/realhand/{side}/arm/pose", pose_callback, 10
                )
            )

    def publish_hand_command(
        self,
        names: list[str],
        positions: list[float] | None = None,
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> None:
        if self._hand_command_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions or []
        msg.velocity = velocities or []
        msg.effort = efforts or []
        self._hand_command_pub.publish(msg)

    def publish_hand_json(self, payload: str) -> None:
        if self._hand_json_pub is None:
            return
        msg = String()
        msg.data = payload
        self._hand_json_pub.publish(msg)

    def publish_arm_command(
        self,
        names: list[str],
        positions: list[float] | None = None,
        velocities: list[float] | None = None,
        efforts: list[float] | None = None,
    ) -> None:
        if self._arm_joint_pub is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions or []
        msg.velocity = velocities or []
        msg.effort = efforts or []
        self._arm_joint_pub.publish(msg)

    def publish_arm_json(self, payload: str) -> None:
        if self._arm_json_pub is None:
            return
        msg = String()
        msg.data = payload
        self._arm_json_pub.publish(msg)
