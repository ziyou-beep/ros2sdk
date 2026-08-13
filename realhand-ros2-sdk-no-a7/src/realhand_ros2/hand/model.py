"""Hand model metadata matching the RealHand Python SDK package layout."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class HandModelSpec:
    """Metadata needed by ROS nodes to expose a RealHand hand model."""

    name: str
    class_name: str
    joint_names: tuple[str, ...]
    supports_speed: bool = True
    supports_torque: bool = True

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)


SIX_DOF_JOINTS = (
    "thumb_flex",
    "thumb_abd",
    "index",
    "middle",
    "ring",
    "pinky",
)

L20_LIKE_JOINTS = (
    "thumb_abd",
    "thumb_yaw",
    "thumb_root1",
    "thumb_tip",
    "index_abd",
    "index_root1",
    "index_tip",
    "middle_abd",
    "middle_root1",
    "middle_tip",
    "ring_abd",
    "ring_root1",
    "ring_tip",
    "pinky_abd",
    "pinky_root1",
    "pinky_tip",
)

L20LITE_JOINTS = (
    "thumb_flex",
    "thumb_abd",
    "index_flex",
    "middle_flex",
    "ring_flex",
    "pinky_flex",
    "index_abd",
    "ring_abd",
    "pinky_abd",
    "thumb_yaw",
)

HAND_MODELS: dict[str, HandModelSpec] = {
    "L6": HandModelSpec(name="L6", class_name="L6", joint_names=SIX_DOF_JOINTS),
    "O6": HandModelSpec(name="O6", class_name="O6", joint_names=SIX_DOF_JOINTS),
    "L20": HandModelSpec(name="L20", class_name="L20", joint_names=L20_LIKE_JOINTS),
    "L20LITE": HandModelSpec(
        name="L20lite", class_name="L20lite", joint_names=L20LITE_JOINTS
    ),
    "L25": HandModelSpec(name="L25", class_name="L25", joint_names=L20_LIKE_JOINTS),
}


def normalize_hand_model(model: str) -> str:
    return model.replace("_", "").replace("-", "").upper()


def get_hand_model_spec(model: str) -> HandModelSpec:
    key = normalize_hand_model(model)
    try:
        return HAND_MODELS[key]
    except KeyError as exc:
        supported = ", ".join(spec.name for spec in HAND_MODELS.values())
        raise ValueError(f"Unsupported hand model {model!r}. Supported: {supported}") from exc


def get_hand_class(model: str) -> type[Any]:
    """Resolve a RealHand Python SDK hand class lazily."""

    from realhand import L6, L20, L20lite, L25, O6

    classes: dict[str, type[Any]] = {
        "L6": L6,
        "O6": O6,
        "L20": L20,
        "L20LITE": L20lite,
        "L25": L25,
    }
    return classes[normalize_hand_model(model)]
