"""Thin adapter around the RealHand Python SDK hand objects."""

from __future__ import annotations

import importlib
import threading
import time
from dataclasses import asdict, is_dataclass
from typing import Any, Iterable

from realhand_ros2.hand.model import HandModelSpec, get_hand_class, get_hand_model_spec


EVENT_MODULES = {
    "L6": "realhand.hand.l6.events",
    "O6": "realhand.hand.o6.events",
    "L20lite": "realhand.hand.l20lite.events",
    "L20": "realhand.hand.l20.events",
    "L25": "realhand.hand.l25.events",
}

# Matches the Python SDK GUI's default sensor/control polling rates.
POLL_INTERVALS_BY_NAME = {
    "angle": 1 / 30,
    "force_sensor": 1 / 15,
    "torque": 0.20,
    "speed": 0.50,
    "acceleration": 0.50,
    "temperature": 1.00,
    "current": 0.50,
    "fault": 1.00,
}


class HandAdapter:
    """Wrap a RealHand hand class behind ROS-friendly operations."""

    def __init__(
        self,
        model: str,
        side: str,
        interface_name: str,
        interface_type: str = "socketcan",
        *,
        poll_on_start: bool = True,
        poll_intervals: dict[str, float] | None = None,
    ) -> None:
        self.spec = get_hand_model_spec(model)
        self.side = side
        self.interface_name = interface_name
        self.interface_type = interface_type
        self._device_info_payload: dict[str, Any] | None = None
        self._device_info_error: str | None = None
        self._control_lock = threading.RLock()
        self._polling_active = False
        self._poll_intervals_by_name: dict[str, float] = {}
        hand_class = get_hand_class(self.spec.name)
        self._hand = hand_class(
            side=side,
            interface_name=interface_name,
            interface_type=interface_type,
        )
        self._read_device_info()
        if poll_on_start:
            self.start_supported_polling(poll_intervals)

    @property
    def model(self) -> HandModelSpec:
        return self.spec

    def close(self) -> None:
        self.stop_stream()
        self.stop_polling()
        close = getattr(self._hand, "close", None)
        if callable(close):
            close()

    def set_angles(self, values: Iterable[float]) -> None:
        with self._control_lock:
            self._hand.angle.set_angles(list(values))

    def set_speeds(self, values: Iterable[float]) -> None:
        speed = getattr(self._hand, "speed", None)
        if speed is None or not hasattr(speed, "set_speeds"):
            raise RuntimeError(f"{self.spec.name} does not expose speed control")
        with self._control_lock:
            speed.set_speeds(list(values))

    def set_torques(self, values: Iterable[float]) -> None:
        torque = getattr(self._hand, "torque", None)
        if torque is None or not hasattr(torque, "set_torques"):
            raise RuntimeError(f"{self.spec.name} does not expose torque control")
        with self._control_lock:
            torque.set_torques(list(values))

    def clear_faults(self) -> None:
        fault = getattr(self._hand, "fault", None)
        clear_faults = getattr(fault, "clear_faults", None)
        if not callable(clear_faults):
            raise RuntimeError(f"{self.spec.name} does not expose fault clearing")
        with self._control_lock:
            clear_faults()

    @property
    def device_info_error(self) -> str | None:
        return self._device_info_error

    def device_info_payload(self) -> dict[str, Any]:
        if self._device_info_payload is not None:
            return dict(self._device_info_payload)
        return self._device_info_unavailable_payload(self._device_info_error)

    def _read_device_info(self) -> None:
        version = getattr(self._hand, "version", None)
        get_device_info = getattr(version, "get_device_info", None)
        if not callable(get_device_info):
            self._device_info_error = f"{self.spec.name} does not expose device info"
            self._device_info_payload = None
            return

        try:
            stop_polling = getattr(self._hand, "stop_polling", None)
            if callable(stop_polling):
                stop_polling()
            info = get_device_info()
        except Exception as exc:
            self._device_info_error = str(exc)
            self._device_info_payload = None
            return

        self._device_info_error = None
        self._device_info_payload = {
            "available": True,
            "model": self.spec.name,
            "side": self.side,
            "interface_name": self.interface_name,
            "interface_type": self.interface_type,
            "serial_number": str(getattr(info, "serial_number", "")),
            "firmware_version": str(getattr(info, "firmware_version", "")),
            "mechanical_version": str(getattr(info, "mechanical_version", "")),
            "pcb_version": str(getattr(info, "pcb_version", "")),
            "timestamp": float(getattr(info, "timestamp", time.time())),
        }

    def _device_info_unavailable_payload(self, error: str | None) -> dict[str, Any]:
        return {
            "available": False,
            "model": self.spec.name,
            "side": self.side,
            "interface_name": self.interface_name,
            "interface_type": self.interface_type,
            "serial_number": "",
            "firmware_version": "",
            "mechanical_version": "",
            "pcb_version": "",
            "timestamp": time.time(),
            "error": error or "device info unavailable",
        }

    def supported_sensor_names(self) -> list[str]:
        sensor_source = self._sensor_source_enum()
        return [source.value for source in sensor_source]

    def polling_status(self) -> dict[str, Any]:
        return {
            "active": self._polling_active,
            "intervals": dict(self._poll_intervals_by_name),
            "supported_sensors": self.supported_sensor_names(),
        }

    def start_supported_polling(
        self,
        intervals_by_name: dict[str, float] | None = None,
        *,
        strict: bool = False,
    ) -> dict[str, float]:
        start_polling = getattr(self._hand, "start_polling", None)
        if not callable(start_polling):
            raise RuntimeError(f"{self.spec.name} does not expose polling")
        intervals, normalized = self._poll_intervals(intervals_by_name, strict=strict)
        if not intervals:
            raise RuntimeError(f"{self.spec.name} has no supported polling sensors")
        with self._control_lock:
            start_polling(intervals)
            self._polling_active = True
            self._poll_intervals_by_name = normalized
        return dict(normalized)

    def stop_polling(self) -> None:
        stop_polling = getattr(self._hand, "stop_polling", None)
        with self._control_lock:
            if callable(stop_polling):
                stop_polling()
            self._polling_active = False

    def _sensor_source_enum(self):
        events_module = importlib.import_module(EVENT_MODULES[self.spec.name])
        return getattr(events_module, "SensorSource")

    def _poll_intervals(
        self,
        intervals_by_name: dict[str, float] | None,
        *,
        strict: bool,
    ):
        sensor_source = self._sensor_source_enum()
        sources_by_name = {source.value: source for source in sensor_source}
        raw = intervals_by_name if intervals_by_name is not None else POLL_INTERVALS_BY_NAME
        intervals = {}
        normalized: dict[str, float] = {}
        for raw_name, raw_interval in raw.items():
            name = "force_sensor" if raw_name == "touch" else str(raw_name)
            source = sources_by_name.get(name)
            if source is None:
                if strict:
                    raise RuntimeError(f"{self.spec.name} does not support polling {name}")
                continue
            interval = float(raw_interval)
            if interval <= 0:
                raise RuntimeError(f"poll interval for {name} must be positive")
            intervals[source] = interval
            normalized[name] = interval
        return intervals, normalized

    def joint_state(self) -> tuple[list[float] | None, list[float] | None, list[float] | None]:
        """Return latest angle, speed, and torque snapshots."""

        snapshot = self._hand.get_snapshot()
        positions = self._extract_vector(getattr(snapshot, "angle", None), "angles")
        velocities = self._extract_vector(getattr(snapshot, "speed", None), "speeds")
        efforts = self._extract_vector(getattr(snapshot, "torque", None), "torques")
        return positions, velocities, efforts

    def snapshot_payload(self) -> dict[str, Any]:
        return self._to_jsonable(self._hand.get_snapshot())

    def sensor_snapshot_payload(self, sensor: str) -> Any:
        name = self._normalize_sensor_name(sensor)
        if name in ("all", "snapshot"):
            return self.snapshot_payload()
        target = getattr(self._hand, name, None)
        get_snapshot = getattr(target, "get_snapshot", None)
        if not callable(get_snapshot):
            raise RuntimeError(f"{self.spec.name} does not expose {name}.get_snapshot")
        return self._to_jsonable(get_snapshot())

    def get_blocking_payload(
        self,
        sensor: str,
        *,
        timeout_ms: float = 1000,
        pause_polling: bool = True,
    ) -> Any:
        name = self._normalize_sensor_name(sensor)
        target = getattr(self._hand, name, None)
        get_blocking = getattr(target, "get_blocking", None)
        if not callable(get_blocking):
            raise RuntimeError(f"{self.spec.name} does not expose {name}.get_blocking")

        restore_intervals = None
        with self._control_lock:
            if pause_polling and self._polling_active:
                restore_intervals = dict(self._poll_intervals_by_name)
                self.stop_polling()
            try:
                return self._to_jsonable(get_blocking(timeout_ms=timeout_ms))
            finally:
                if restore_intervals is not None:
                    self.start_supported_polling(restore_intervals)

    def force_sensor_payload(self) -> dict[str, Any] | None:
        force_sensor = getattr(self._hand, "force_sensor", None)
        if force_sensor is None:
            return None

        get_snapshot = getattr(force_sensor, "get_snapshot", None)
        if callable(get_snapshot):
            snapshot = get_snapshot()
            if snapshot is not None:
                return self._to_jsonable(snapshot)

        get_finger = getattr(force_sensor, "get_finger", None)
        if not callable(get_finger):
            return None

        payload: dict[str, Any] = {}
        for name in ("thumb", "index", "middle", "ring", "pinky"):
            try:
                finger = get_finger(name)
            except Exception:
                continue
            finger_snapshot = getattr(finger, "get_snapshot", None)
            if not callable(finger_snapshot):
                continue
            data = finger_snapshot()
            if data is not None:
                payload[name] = self._to_jsonable(data)
        return payload or None

    def stream_events(self, maxsize: int = 300):
        stream = getattr(self._hand, "stream", None)
        if not callable(stream):
            return None
        return stream(maxsize=maxsize)

    def stop_stream(self) -> None:
        stop_stream = getattr(self._hand, "stop_stream", None)
        if callable(stop_stream):
            stop_stream()

    @staticmethod
    def _normalize_sensor_name(sensor: str) -> str:
        name = str(sensor).strip().lower()
        if name == "touch":
            return "force_sensor"
        return name

    @staticmethod
    def _extract_vector(data: Any, field_name: str) -> list[float] | None:
        if data is None:
            return None
        value = getattr(data, field_name, None)
        if value is None:
            return None
        if hasattr(value, "to_list"):
            return [float(item) for item in value.to_list()]
        if hasattr(value, "tolist"):
            return [float(item) for item in value.tolist()]
        if isinstance(value, (list, tuple)):
            return [float(item) for item in value]
        return None

    @classmethod
    def _to_jsonable(cls, value: Any) -> Any:
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        if is_dataclass(value):
            return cls._to_jsonable(asdict(value))
        if hasattr(value, "model_dump"):
            return cls._to_jsonable(value.model_dump())
        if hasattr(value, "to_list"):
            return value.to_list()
        if hasattr(value, "tolist"):
            return cls._to_jsonable(value.tolist())
        if isinstance(value, dict):
            return {str(k): cls._to_jsonable(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._to_jsonable(item) for item in value]
        return repr(value)
