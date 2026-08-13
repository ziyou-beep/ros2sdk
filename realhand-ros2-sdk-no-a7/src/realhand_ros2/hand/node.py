"""ROS2 node for RealHand Python SDK hand classes."""

from __future__ import annotations

import json
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from realhand_ros2.hand.adapter import HandAdapter


class HandNode(Node):
    """Expose a RealHand Python SDK hand as ROS2 topics."""

    def __init__(self) -> None:
        super().__init__("realhand_hand_node")

        self.declare_parameter("model", "L6")
        self.declare_parameter("side", "left")
        self.declare_parameter("interface_name", "can0")
        self.declare_parameter("interface_type", "socketcan")
        self.declare_parameter("connect_on_start", True)
        self.declare_parameter("state_rate_hz", 30.0)
        self.declare_parameter("fill_missing_fields", True)
        self.declare_parameter("poll_on_start", True)
        self.declare_parameter("stream_on_start", True)
        self.declare_parameter("stream_queue_size", 300)
        self.declare_parameter("poll_intervals_json", "")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("json_command_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("snapshot_topic", "")
        self.declare_parameter("device_info_topic", "")
        self.declare_parameter("angle_topic", "")
        self.declare_parameter("speed_topic", "")
        self.declare_parameter("torque_topic", "")
        self.declare_parameter("acceleration_topic", "")
        self.declare_parameter("temperature_topic", "")
        self.declare_parameter("current_topic", "")
        self.declare_parameter("touch_topic", "")
        self.declare_parameter("force_sensor_topic", "")
        self.declare_parameter("fault_topic", "")
        self.declare_parameter("blocking_result_topic", "")
        self.declare_parameter("control_status_topic", "")

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.side = self.get_parameter("side").get_parameter_value().string_value
        self.interface_name = (
            self.get_parameter("interface_name").get_parameter_value().string_value
        )
        self.interface_type = (
            self.get_parameter("interface_type").get_parameter_value().string_value
        )
        self.fill_missing_fields = (
            self.get_parameter("fill_missing_fields").get_parameter_value().bool_value
        )
        self.poll_on_start = (
            self.get_parameter("poll_on_start").get_parameter_value().bool_value
        )
        self.stream_on_start = (
            self.get_parameter("stream_on_start").get_parameter_value().bool_value
        )
        self.stream_queue_size = (
            self.get_parameter("stream_queue_size").get_parameter_value().integer_value
        )
        self.poll_intervals = self._parse_poll_intervals_json(
            self.get_parameter("poll_intervals_json").get_parameter_value().string_value
        )
        state_rate_hz = self.get_parameter("state_rate_hz").value

        command_topic = self._topic_param(
            "command_topic", f"/realhand/{self.side}/hand/command"
        )
        json_command_topic = self._topic_param(
            "json_command_topic", f"/realhand/{self.side}/hand/command_json"
        )
        state_topic = self._topic_param(
            "state_topic", f"/realhand/{self.side}/hand/state"
        )
        snapshot_topic = self._topic_param(
            "snapshot_topic", f"/realhand/{self.side}/hand/snapshot"
        )
        device_info_topic = self._topic_param(
            "device_info_topic", f"/realhand/{self.side}/hand/device_info"
        )
        angle_topic = self._topic_param(
            "angle_topic", f"/realhand/{self.side}/hand/angle"
        )
        speed_topic = self._topic_param(
            "speed_topic", f"/realhand/{self.side}/hand/speed"
        )
        torque_topic = self._topic_param(
            "torque_topic", f"/realhand/{self.side}/hand/torque"
        )
        acceleration_topic = self._topic_param(
            "acceleration_topic", f"/realhand/{self.side}/hand/acceleration"
        )
        temperature_topic = self._topic_param(
            "temperature_topic", f"/realhand/{self.side}/hand/temperature"
        )
        current_topic = self._topic_param(
            "current_topic", f"/realhand/{self.side}/hand/current"
        )
        touch_topic = self._topic_param(
            "touch_topic", f"/realhand/{self.side}/hand/touch"
        )
        force_sensor_topic = self._topic_param(
            "force_sensor_topic", f"/realhand/{self.side}/hand/force_sensor"
        )
        fault_topic = self._topic_param(
            "fault_topic", f"/realhand/{self.side}/hand/fault"
        )
        blocking_result_topic = self._topic_param(
            "blocking_result_topic", f"/realhand/{self.side}/hand/blocking_result"
        )
        control_status_topic = self._topic_param(
            "control_status_topic", f"/realhand/{self.side}/hand/control_status"
        )

        self.adapter: HandAdapter | None = None
        self._stream_stop = threading.Event()
        self._stream_thread: threading.Thread | None = None
        self._stream_active = False
        if self.get_parameter("connect_on_start").get_parameter_value().bool_value:
            self.adapter = HandAdapter(
                model=self.model,
                side=self.side,
                interface_name=self.interface_name,
                interface_type=self.interface_type,
                poll_on_start=self.poll_on_start,
                poll_intervals=self.poll_intervals,
            )
            self.get_logger().info(
                f"Connected {self.model} {self.side} hand on {self.interface_name}"
            )
            if self.adapter.device_info_error is not None:
                self.get_logger().warning(
                    f"Device info unavailable: {self.adapter.device_info_error}"
                )
            else:
                self.get_logger().info("Device info loaded")
        else:
            self.get_logger().warning("connect_on_start is false; hardware is not open")

        self.state_pub = self.create_publisher(JointState, state_topic, 10)
        self.snapshot_pub = self.create_publisher(String, snapshot_topic, 10)
        self.device_info_pub = self.create_publisher(String, device_info_topic, 10)
        self.angle_pub = self.create_publisher(String, angle_topic, 10)
        self.speed_pub = self.create_publisher(String, speed_topic, 10)
        self.torque_pub = self.create_publisher(String, torque_topic, 10)
        self.acceleration_pub = self.create_publisher(String, acceleration_topic, 10)
        self.temperature_pub = self.create_publisher(String, temperature_topic, 10)
        self.current_pub = self.create_publisher(String, current_topic, 10)
        self.touch_pub = self.create_publisher(String, touch_topic, 10)
        self.force_sensor_pub = self.create_publisher(String, force_sensor_topic, 10)
        self.fault_pub = self.create_publisher(String, fault_topic, 10)
        self.blocking_result_pub = self.create_publisher(
            String, blocking_result_topic, 10
        )
        self.control_status_pub = self.create_publisher(String, control_status_topic, 10)
        self.create_subscription(JointState, command_topic, self._on_joint_command, 10)
        self.create_subscription(String, json_command_topic, self._on_json_command, 10)

        period = 1.0 / float(state_rate_hz) if state_rate_hz else 1.0 / 30.0
        self.create_timer(period, self._publish_state)
        self.create_timer(1.0, self._publish_device_info)
        self._publish_device_info()
        if self.stream_on_start:
            self._start_sensor_stream(publish_status=False)

    def _topic_param(self, name: str, fallback: str) -> str:
        value = self.get_parameter(name).get_parameter_value().string_value
        return value or fallback

    def _parse_poll_intervals_json(self, value: str) -> dict[str, float] | None:
        if not value:
            return None
        payload = json.loads(value)
        return self._parse_poll_intervals(payload)

    def _parse_poll_intervals(self, payload: Any) -> dict[str, float]:
        if not isinstance(payload, dict):
            raise ValueError("poll intervals must be a JSON object")
        intervals = {}
        for key, value in payload.items():
            interval = float(value)
            if interval <= 0:
                raise ValueError(f"poll interval for {key!r} must be positive")
            intervals[str(key)] = interval
        return intervals

    def _on_joint_command(self, msg: JointState) -> None:
        if self.adapter is None:
            self.get_logger().warning("Ignoring hand command; hardware is not connected")
            return
        try:
            if msg.velocity:
                self.adapter.set_speeds(msg.velocity)
            if msg.effort:
                self.adapter.set_torques(msg.effort)
            if msg.position:
                self.adapter.set_angles(msg.position)
        except Exception as exc:
            self.get_logger().error(f"Hand command failed: {exc}")

    def _on_json_command(self, msg: String) -> None:
        if self.adapter is None:
            self.get_logger().warning("Ignoring JSON command; hardware is not connected")
            return
        try:
            payload = json.loads(msg.data)
            self._apply_json_command(payload)
        except Exception as exc:
            self.get_logger().error(f"JSON hand command failed: {exc}")

    def _apply_json_command(self, payload: dict[str, Any]) -> None:
        if self.adapter is None:
            return
        action = payload.get("action")
        if action == "clear_faults":
            self.adapter.clear_faults()
            self._publish_control_status(action, True)
        elif action == "start_polling":
            intervals = (
                self._parse_poll_intervals(payload["intervals"])
                if "intervals" in payload
                else self.poll_intervals
            )
            active_intervals = self.adapter.start_supported_polling(
                intervals, strict=intervals is not None
            )
            self._publish_control_status(
                action, True, polling_active=True, intervals=active_intervals
            )
        elif action == "stop_polling":
            self.adapter.stop_polling()
            self._publish_control_status(action, True, polling_active=False)
        elif action == "start_stream":
            maxsize = int(payload.get("maxsize", self.stream_queue_size))
            self._start_sensor_stream(maxsize=maxsize)
        elif action == "stop_stream":
            self._stop_sensor_stream()
        elif action == "get_blocking":
            self._start_blocking_read(payload)
        elif action == "get_snapshot":
            self._publish_snapshot_result(payload)
        elif action not in (None, ""):
            raise RuntimeError(f"unsupported hand JSON action: {action}")
        if "speeds" in payload:
            self.adapter.set_speeds(payload["speeds"])
        if "torques" in payload:
            self.adapter.set_torques(payload["torques"])
        if "angles" in payload:
            self.adapter.set_angles(payload["angles"])

    def _publish_state(self) -> None:
        if self.adapter is None:
            return
        try:
            positions, velocities, efforts = self.adapter.joint_state()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.adapter.model.joint_names)
            msg.position = self._field_or_default(positions)
            msg.velocity = self._field_or_default(velocities)
            msg.effort = self._field_or_default(efforts)
            self.state_pub.publish(msg)

            snapshot = self.adapter.snapshot_payload()
            self._publish_json(self.snapshot_pub, snapshot)
            force_sensor = snapshot.get("force_sensor") or self.adapter.force_sensor_payload()
            self._publish_sensor_topics(snapshot, force_sensor)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish hand state: {exc}")

    def _publish_sensor_topics(
        self, snapshot: dict[str, Any], force_sensor: Any
    ) -> None:
        self._publish_optional_json(self.angle_pub, snapshot.get("angle"))
        self._publish_optional_json(self.speed_pub, snapshot.get("speed"))
        self._publish_optional_json(self.torque_pub, snapshot.get("torque"))
        self._publish_optional_json(self.acceleration_pub, snapshot.get("acceleration"))
        self._publish_optional_json(self.temperature_pub, snapshot.get("temperature"))
        self._publish_optional_json(self.current_pub, snapshot.get("current"))
        self._publish_optional_json(self.fault_pub, snapshot.get("fault"))

        self._publish_optional_json(self.force_sensor_pub, force_sensor)
        self._publish_optional_json(self.touch_pub, force_sensor)

    def _publish_device_info(self) -> None:
        if self.adapter is None:
            return
        self._publish_json(self.device_info_pub, self.adapter.device_info_payload())

    def _publish_optional_json(self, publisher, payload: Any) -> None:
        if payload is not None:
            self._publish_json(publisher, payload)

    def _publish_json(self, publisher, payload: Any) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        publisher.publish(msg)

    def _start_sensor_stream(
        self,
        *,
        maxsize: int | None = None,
        publish_status: bool = True,
    ) -> None:
        if self.adapter is None:
            return
        if self._stream_thread is not None and self._stream_thread.is_alive():
            self._stop_sensor_stream(publish_status=False)
        queue_size = maxsize if maxsize is not None else self.stream_queue_size
        if queue_size <= 0:
            raise RuntimeError("stream maxsize must be positive")
        queue = self.adapter.stream_events(maxsize=queue_size)
        if queue is None:
            if publish_status:
                self._publish_control_status(
                    "start_stream", False, error="stream is not supported"
                )
            return
        self._stream_stop.clear()
        self._stream_active = True

        def run() -> None:
            try:
                for event in queue:
                    if self._stream_stop.is_set():
                        break
                    self._handle_sensor_event(event)
            except Exception as exc:
                if not self._stream_stop.is_set():
                    self.get_logger().error(f"Sensor stream stopped: {exc}")
            finally:
                self._stream_active = False

        self._stream_thread = threading.Thread(
            target=run,
            name=f"{self.model}-ros2-sensor-stream",
            daemon=True,
        )
        self._stream_thread.start()
        if publish_status:
            self._publish_control_status(
                "start_stream", True, streaming=True, maxsize=queue_size
            )

    def _handle_sensor_event(self, event: Any) -> None:
        if self.adapter is None:
            return
        name = type(event).__name__
        data = getattr(event, "data", None)
        payload = self.adapter._to_jsonable(data)

        if name == "ForceSensorEvent":
            self._publish_optional_json(self.force_sensor_pub, payload)
            self._publish_optional_json(self.touch_pub, payload)
            return

        publisher = {
            "AngleEvent": self.angle_pub,
            "SpeedEvent": self.speed_pub,
            "TorqueEvent": self.torque_pub,
            "AccelerationEvent": self.acceleration_pub,
            "TemperatureEvent": self.temperature_pub,
            "CurrentEvent": self.current_pub,
            "FaultEvent": self.fault_pub,
        }.get(name)
        if publisher is not None:
            self._publish_optional_json(publisher, payload)

    def _stop_sensor_stream(self, *, publish_status: bool = True) -> None:
        self._stream_stop.set()
        if self.adapter is not None:
            self.adapter.stop_stream()
        if self._stream_thread is not None and self._stream_thread.is_alive():
            self._stream_thread.join(timeout=1.0)
        self._stream_thread = None
        self._stream_active = False
        if publish_status:
            self._publish_control_status("stop_stream", True, streaming=False)

    def _start_blocking_read(self, payload: dict[str, Any]) -> None:
        sensor = str(payload.get("sensor", ""))
        if not sensor:
            raise RuntimeError("get_blocking requires a sensor field")
        timeout_ms = float(payload.get("timeout_ms", 1000))
        if timeout_ms <= 0:
            raise RuntimeError("timeout_ms must be positive")
        request_id = str(payload.get("request_id", ""))
        pause_polling = self._bool_payload(payload.get("pause_polling", True))

        def run() -> None:
            result = {
                "action": "get_blocking",
                "request_id": request_id,
                "sensor": sensor,
                "timeout_ms": timeout_ms,
                "pause_polling": pause_polling,
                "timestamp": time.time(),
            }
            try:
                if self.adapter is None:
                    raise RuntimeError("hardware is not connected")
                result["data"] = self.adapter.get_blocking_payload(
                    sensor,
                    timeout_ms=timeout_ms,
                    pause_polling=pause_polling,
                )
                result["ok"] = True
            except Exception as exc:
                result["ok"] = False
                result["error"] = str(exc)
            self._publish_json(self.blocking_result_pub, result)

        thread = threading.Thread(
            target=run,
            name=f"{self.model}-ros2-get-blocking-{sensor}",
            daemon=True,
        )
        thread.start()
        self._publish_control_status(
            "get_blocking",
            True,
            accepted=True,
            request_id=request_id,
            sensor=sensor,
            timeout_ms=timeout_ms,
        )

    def _publish_snapshot_result(self, payload: dict[str, Any]) -> None:
        sensor = str(payload.get("sensor", "all"))
        request_id = str(payload.get("request_id", ""))
        result = {
            "action": "get_snapshot",
            "request_id": request_id,
            "sensor": sensor,
            "timestamp": time.time(),
        }
        try:
            if self.adapter is None:
                raise RuntimeError("hardware is not connected")
            result["data"] = self.adapter.sensor_snapshot_payload(sensor)
            result["ok"] = True
        except Exception as exc:
            result["ok"] = False
            result["error"] = str(exc)
        self._publish_json(self.blocking_result_pub, result)

    def _publish_control_status(
        self, action: str, ok: bool, **fields: Any
    ) -> None:
        payload = {
            "action": action,
            "ok": ok,
            "timestamp": time.time(),
            "streaming": self._stream_active,
        }
        if self.adapter is not None:
            payload["polling"] = self.adapter.polling_status()
        payload.update(fields)
        self._publish_json(self.control_status_pub, payload)

    @staticmethod
    def _bool_payload(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _field_or_default(self, values: list[float] | None) -> list[float]:
        if values is not None:
            return values
        if not self.fill_missing_fields or self.adapter is None:
            return []
        return [0.0] * self.adapter.model.joint_count

    def destroy_node(self) -> bool:
        self._stop_sensor_stream()
        if self.adapter is not None:
            self.adapter.close()
            self.adapter = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
