"""RealHand CAN autodetection helpers for the ROS2 GUI."""

from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Any

from realhand_ros2.hand.model import HAND_MODELS


DEFAULT_CAN_BITRATE = 1_000_000
CAN_PROBE_INTERFACES = tuple(f"can{index}" for index in range(4))
PROBE_SIDES = (
    ("right", 0x27),
    ("left", 0x28),
)
SERIAL_NUMBER_CMD = 0xC0
SERIAL_TIMEOUT_S = 1.0

SERIAL_MODEL_CODES = {
    "L6": "L6",
    "T6": "L6",
    "O6": "O6",
    "L20": "L20",
    "T20": "L20",
    "L20LITE": "L20lite",
    "L25": "L25",
}
SERIAL_SIDE_CODES = {
    "L": "left",
    "R": "right",
}


@dataclass(frozen=True)
class InterfaceSetupResult:
    should_probe: bool
    message: str


@dataclass(frozen=True)
class DetectedHand:
    interface: str
    interface_type: str
    model: str
    side: str
    serial_number: str
    arbitration_side: str

    def label(self) -> str:
        return (
            f"{self.interface}: {self.side} {self.model} "
            f"(serial {self.serial_number}, arbitration {self.arbitration_side})"
        )


def parse_serial_identity(serial_number: str) -> dict[str, str]:
    sections = serial_number.strip().split("-") if serial_number else []
    serial_prefix = sections[0].strip() if len(sections) >= 1 else ""
    section_2 = sections[1].strip() if len(sections) >= 2 else ""
    section_3 = sections[2].strip() if len(sections) >= 3 else ""
    side_code = sections[3].strip().upper() if len(sections) >= 4 else ""
    remaining = "-".join(section.strip() for section in sections[4:])

    model_code = serial_prefix.upper()
    if model_code.startswith("LH"):
        model_code = model_code[2:]

    return {
        "serial_model": SERIAL_MODEL_CODES.get(model_code, model_code),
        "serial_side": SERIAL_SIDE_CODES.get(side_code, ""),
        "serial_prefix": serial_prefix,
        "serial_section_2": section_2,
        "serial_section_3": section_3,
        "serial_side_code": side_code,
        "serial_remaining": remaining,
    }


def _run_ip_link(
    args: list[str], *, timeout_s: float = 2.0
) -> subprocess.CompletedProcess[str] | None:
    try:
        return subprocess.run(
            ["ip", "link", *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_s,
        )
    except FileNotFoundError:
        return None


def configure_socketcan_interface(
    interface: str, bitrate: int, *, enabled: bool
) -> InterfaceSetupResult:
    if not enabled:
        return InterfaceSetupResult(True, f"{interface}: CAN setup skipped")

    exists = _run_ip_link(["show", interface])
    if exists is None:
        return InterfaceSetupResult(
            True, f"{interface}: ip command not found; probing without link setup"
        )
    if exists.returncode != 0:
        return InterfaceSetupResult(False, f"{interface}: not present")

    commands = (
        ["set", interface, "down"],
        ["set", interface, "type", "can", "bitrate", str(bitrate)],
        ["set", interface, "up"],
    )
    for command in commands:
        result = _run_ip_link(command)
        if result is None:
            return InterfaceSetupResult(
                True, f"{interface}: ip command not found; probing without link setup"
            )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            return InterfaceSetupResult(
                True,
                f"{interface}: setup failed ({detail or 'unknown ip link error'}); probing anyway",
            )
    return InterfaceSetupResult(True, f"{interface}: up at {bitrate} bps")


def query_can_value(
    *,
    dispatcher: Any,
    arbitration_id: int,
    request_data: list[int],
    handler: Any,
    timeout_s: float,
) -> Any:
    import can

    condition = threading.Condition()
    result: dict[str, Any] = {"ready": False, "value": None}

    def callback(msg: Any) -> None:
        value = handler(msg)
        if value is None:
            return
        with condition:
            result["ready"] = True
            result["value"] = value
            condition.notify()

    dispatcher.subscribe(callback)
    try:
        dispatcher.send(
            can.Message(
                arbitration_id=arbitration_id,
                data=request_data,
                is_extended_id=False,
            )
        )
        deadline = time.monotonic() + timeout_s
        with condition:
            while not result["ready"]:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                condition.wait(remaining)
        if not result["ready"]:
            raise TimeoutError("request timed out")
        return result["value"]
    finally:
        dispatcher.unsubscribe(callback)


def decode_standard_serial(frames: dict[int, bytes]) -> str:
    data = bytearray()
    for frame_id in range(4):
        data.extend(frames[frame_id])
    return data.rstrip(b"\x00").decode("ascii", errors="ignore")


def decode_indexed_serial(frames: dict[int, bytes]) -> str:
    data = bytearray(24)
    for byte_index, frame_data in frames.items():
        for offset, value in enumerate(frame_data):
            if byte_index + offset < len(data):
                data[byte_index + offset] = value
    return data.rstrip(b"\x00").decode("ascii", errors="ignore")


def read_serial_number(
    dispatcher: Any,
    arbitration_id: int,
    *,
    timeout_s: float = SERIAL_TIMEOUT_S,
) -> str:
    standard_frames: dict[int, bytes] = {}
    indexed_frames: dict[int, bytes] = {}

    def handler(msg: Any) -> str | None:
        if msg.arbitration_id != arbitration_id:
            return None
        if len(msg.data) < 2 or msg.data[0] != SERIAL_NUMBER_CMD:
            return None

        frame_key = int(msg.data[1])
        frame_data = bytes(msg.data[2:8])
        if frame_key in range(4):
            standard_frames[frame_key] = frame_data
            if all(index in standard_frames for index in range(4)):
                return decode_standard_serial(standard_frames)

        if frame_key in (0, 6, 12, 18):
            indexed_frames[frame_key] = frame_data
            if all(index in indexed_frames for index in (0, 6, 12, 18)):
                return decode_indexed_serial(indexed_frames)

        return None

    return query_can_value(
        dispatcher=dispatcher,
        arbitration_id=arbitration_id,
        request_data=[SERIAL_NUMBER_CMD],
        handler=handler,
        timeout_s=timeout_s,
    )


def detect_hands_on_interface(
    *,
    interface: str,
    interface_type: str,
) -> tuple[list[DetectedHand], list[str]]:
    from realhand.comm import CANMessageDispatcher

    detected: list[DetectedHand] = []
    messages: list[str] = []
    dispatcher = None

    try:
        dispatcher = CANMessageDispatcher(
            interface_name=interface, interface_type=interface_type
        )
    except Exception as exc:
        return detected, [f"{interface}: open failed: {exc}"]

    try:
        for arbitration_side, arbitration_id in PROBE_SIDES:
            try:
                serial_number = read_serial_number(dispatcher, arbitration_id)
            except Exception as exc:
                messages.append(
                    f"{interface}/{arbitration_side}: no serial response ({exc})"
                )
                continue

            identity = parse_serial_identity(serial_number)
            model = identity["serial_model"]
            side = identity["serial_side"] or arbitration_side
            if model.upper() not in HAND_MODELS:
                messages.append(
                    f"{interface}/{arbitration_side}: unsupported model "
                    f"{model or 'unknown'} from serial {serial_number}"
                )
                continue
            if side not in ("left", "right"):
                messages.append(
                    f"{interface}/{arbitration_side}: unknown side in serial "
                    f"{serial_number}; using arbitration side {arbitration_side}"
                )
                side = arbitration_side

            detected.append(
                DetectedHand(
                    interface=interface,
                    interface_type=interface_type,
                    model=HAND_MODELS[model.upper()].name,
                    side=side,
                    serial_number=serial_number,
                    arbitration_side=arbitration_side,
                )
            )
    finally:
        if dispatcher is not None:
            try:
                dispatcher.stop()
            except Exception:
                pass

    return detected, messages


def autodetect_hands(
    *,
    interface_type: str = "socketcan",
    bitrate: int = DEFAULT_CAN_BITRATE,
    setup_can: bool = True,
    interfaces: tuple[str, ...] = CAN_PROBE_INTERFACES,
) -> tuple[list[DetectedHand], list[str]]:
    detected: list[DetectedHand] = []
    messages: list[str] = []
    seen: set[tuple[str, str, str]] = set()

    for interface in interfaces:
        if interface_type == "socketcan":
            setup = configure_socketcan_interface(interface, bitrate, enabled=setup_can)
            messages.append(setup.message)
            if not setup.should_probe:
                continue

        interface_hands, interface_messages = detect_hands_on_interface(
            interface=interface,
            interface_type=interface_type,
        )
        messages.extend(interface_messages)
        for hand in interface_hands:
            key = (hand.interface, hand.side, hand.serial_number)
            if key in seen:
                continue
            seen.add(key)
            detected.append(hand)

    return detected, messages
