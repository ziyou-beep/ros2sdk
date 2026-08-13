"""Arm model metadata matching the RealHand Python SDK package layout."""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ArmModelSpec:
    name: str
    class_name: str
    joint_names: tuple[str, ...]
    default_interface_name: str = "can0"
    default_interface_type: str = "socketcan"
    velocity_range: tuple[float, float] = (-10.0, 10.0)
    acceleration_range: tuple[float, float] = (0.0, 100.0)

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)


ARM_JOINTS = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
)

ARM_MODELS: dict[str, ArmModelSpec] = {
    "A7LITE": ArmModelSpec(name="A7lite", class_name="A7lite", joint_names=ARM_JOINTS),
    "P7": ArmModelSpec(
        name="P7",
        class_name="P7",
        joint_names=ARM_JOINTS,
        default_interface_name="192.168.10.21",
        default_interface_type="lbot",
        velocity_range=(0.0, 20.0),
        acceleration_range=(0.0, 20.0),
    ),
}


def normalize_arm_model(model: str) -> str:
    return model.replace("_", "").replace("-", "").upper()


def get_arm_model_spec(model: str) -> ArmModelSpec:
    key = normalize_arm_model(model)
    try:
        return ARM_MODELS[key]
    except KeyError as exc:
        supported = ", ".join(spec.name for spec in ARM_MODELS.values())
        raise ValueError(f"Unsupported arm model {model!r}. Supported: {supported}") from exc


def get_arm_class(model: str) -> type[Any]:
    spec = get_arm_model_spec(model)
    module = importlib.import_module("realhand.arm")
    try:
        return getattr(module, spec.class_name)
    except AttributeError as exc:
        raise ImportError(
            f"Python SDK package realhand.arm does not expose {spec.class_name}. "
            "Install an SDK version that supports this arm model."
        ) from exc
