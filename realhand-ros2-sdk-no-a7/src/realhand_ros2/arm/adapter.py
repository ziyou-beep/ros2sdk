"""Thin adapter around the RealHand Python SDK arm objects."""

from __future__ import annotations

from typing import Any, Iterable

from realhand_ros2.arm.model import ArmModelSpec, get_arm_class, get_arm_model_spec


class ArmAdapter:
    """Wrap a RealHand arm behind ROS-friendly operations."""

    def __init__(
        self,
        model: str,
        side: str,
        interface_name: str,
        interface_type: str = "socketcan",
        tcp_offset: list[float] | None = None,
        world_frame: str = "urdf",
    ) -> None:
        self.spec = get_arm_model_spec(model)
        arm_class = get_arm_class(self.spec.name)
        kwargs: dict[str, Any] = {
            "side": side,
            "interface_name": interface_name,
            "interface_type": interface_type,
            "world_frame": world_frame,
        }
        if tcp_offset is not None:
            kwargs["tcp_offset"] = tcp_offset
        self._arm = arm_class(**kwargs)

    @property
    def model(self) -> ArmModelSpec:
        return self.spec

    def close(self) -> None:
        close = getattr(self._arm, "close", None)
        if callable(close):
            close()

    def move_joints(self, values: Iterable[float], *, blocking: bool = False) -> None:
        self._arm.move_j(list(values), blocking=blocking)

    def set_velocities(self, values: Iterable[float]) -> None:
        self._arm.set_velocities(list(values))

    def set_accelerations(self, values: Iterable[float]) -> None:
        self._arm.set_accelerations(list(values))

    def enable(self) -> None:
        self._arm.enable()

    def disable(self) -> None:
        self._arm.disable()

    def home(self, *, blocking: bool = False) -> None:
        self._arm.home(blocking=blocking)

    def reset_error(self) -> None:
        self._arm.reset_error()

    def emergency_stop(self, enable: bool = True) -> None:
        self._arm.emergency_stop(enable)

    def resume_from_emergency_stop(self) -> None:
        resume = getattr(self._arm, "resume_from_emergency_stop", None)
        if callable(resume):
            resume()
            return
        emergency_stop = getattr(self._arm, "emergency_stop", None)
        if callable(emergency_stop):
            emergency_stop(False)
            return
        raise AttributeError(f"{type(self._arm).__name__} does not support e-stop resume")

    def joint_state(self) -> tuple[list[float], list[float], list[float]]:
        positions = [float(value) for value in self._arm.get_angles()]
        velocities = [float(value) for value in self._arm.get_velocities()]
        efforts = [float(value) for value in self._arm.get_torques()]
        return positions, velocities, efforts

    def _optional_float_list(self, method_name: str) -> list[float] | None:
        method = getattr(self._arm, method_name, None)
        if not callable(method):
            return None
        values = method()
        return [float(value) for value in values]

    def control_angles(self) -> list[float] | None:
        return self._optional_float_list("get_control_angles")

    def control_velocities(self) -> list[float] | None:
        return self._optional_float_list("get_control_velocities")

    def control_accelerations(self) -> list[float] | None:
        return self._optional_float_list("get_control_acceleration")

    def temperatures(self) -> list[float] | None:
        return self._optional_float_list("get_temperatures")

    def is_moving(self) -> bool | None:
        method = getattr(self._arm, "is_moving", None)
        if not callable(method):
            return None
        return bool(method())

    def joint_limits(self) -> list[tuple[float, float]] | None:
        method = getattr(self._arm, "get_joint_limits", None)
        if not callable(method):
            return None
        limits = method()
        return [(float(lower), float(upper)) for lower, upper in limits]

    def state_payload(self) -> dict[str, object]:
        payload: dict[str, object] = {
            "angles": self.joint_state()[0],
            "velocities": self.joint_state()[1],
            "efforts": self.joint_state()[2],
            "pose": self.pose_payload(),
        }
        control_angles = self.control_angles()
        if control_angles is not None:
            payload["control_angles"] = control_angles
        control_velocities = self.control_velocities()
        if control_velocities is not None:
            payload["control_velocities"] = control_velocities
        control_accelerations = self.control_accelerations()
        if control_accelerations is not None:
            payload["control_accelerations"] = control_accelerations
        temperatures = self.temperatures()
        if temperatures is not None:
            payload["temperatures"] = temperatures
        is_moving = self.is_moving()
        if is_moving is not None:
            payload["is_moving"] = is_moving
        joint_limits = self.joint_limits()
        if joint_limits is not None:
            payload["joint_limits"] = [
                {"lower": lower, "upper": upper} for lower, upper in joint_limits
            ]
        return payload

    def pose_payload(self) -> dict[str, float]:
        pose = self._arm.get_pose()
        return {
            "x": float(pose.x),
            "y": float(pose.y),
            "z": float(pose.z),
            "rx": float(pose.rx),
            "ry": float(pose.ry),
            "rz": float(pose.rz),
        }
