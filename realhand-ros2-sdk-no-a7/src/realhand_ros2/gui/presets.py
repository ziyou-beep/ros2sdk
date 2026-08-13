"""GUI preset helpers shared by the hand control panel."""

from __future__ import annotations

from dataclasses import dataclass, field

from realhand_ros2.hand.model import get_hand_model_spec


@dataclass(frozen=True)
class GuiHandConfig:
    joint_names: list[str] = field(default_factory=list)
    init_pos: list[int] = field(default_factory=list)
    preset_actions: dict[str, list[int]] = field(default_factory=dict)
    cycle_loop_actions: list[str] = field(default_factory=list)
    cycle_loop_repeats: int = 0


def _scale_255(values: list[int]) -> list[int]:
    return [round(max(0, min(255, value)) * 100 / 255) for value in values]


DEFAULT_HAND_CONFIGS: dict[str, GuiHandConfig] = {
    "L6": GuiHandConfig(
        joint_names=[
            "Thumb flexion",
            "Thumb yaw",
            "Index finger flexion",
            "Middle finger flexion",
            "Ring finger flexion",
            "Pinky finger flexion",
        ],
        init_pos=_scale_255([250, 250, 250, 250, 250, 250]),
        preset_actions={
            "Open": _scale_255([250, 250, 250, 250, 250, 250]),
            "One": _scale_255([0, 18, 255, 0, 0, 0]),
            "Two": _scale_255([0, 39, 255, 255, 0, 0]),
            "Three": _scale_255([0, 39, 255, 255, 255, 0]),
            "Four": _scale_255([0, 0, 255, 255, 255, 255]),
            "Five": _scale_255([255, 255, 255, 255, 255, 255]),
            "OK": _scale_255([74, 13, 153, 255, 255, 255]),
            "Thumbs Up": _scale_255([255, 255, 0, 0, 0, 0]),
            "Fist": _scale_255([79, 11, 0, 0, 0, 0]),
        },
    ),
    "O6": GuiHandConfig(
        joint_names=[
            "Thumb flexion",
            "Thumb yaw",
            "Index finger flexion",
            "Middle finger flexion",
            "Ring finger flexion",
            "Pinky finger flexion",
        ],
        init_pos=_scale_255([250, 250, 250, 250, 250, 250]),
        preset_actions={
            "Open": _scale_255([250, 250, 250, 250, 250, 250]),
            "One": _scale_255([125, 18, 255, 0, 0, 0]),
            "Two": _scale_255([92, 87, 255, 255, 0, 0]),
            "Three": _scale_255([92, 87, 255, 255, 255, 0]),
            "Four": _scale_255([92, 87, 255, 255, 255, 255]),
            "Five": _scale_255([255, 255, 255, 255, 255, 255]),
            "OK": _scale_255([96, 100, 118, 250, 250, 250]),
            "Thumbs Up": _scale_255([250, 79, 0, 0, 0, 0]),
            "Fist": _scale_255([102, 18, 0, 0, 0, 0]),
        },
    ),
    "L20lite": GuiHandConfig(
        joint_names=[
            "Thumb flexion",
            "Thumb abduction",
            "Index finger flexion",
            "Middle finger flexion",
            "Ring finger flexion",
            "Pinky finger flexion",
            "Index finger abduction",
            "Ring finger abduction",
            "Pinky finger abduction",
            "Thumb yaw",
        ],
        init_pos=[100, 100, 100, 100, 100, 100, 50, 50, 50, 100],
        preset_actions={
            "Open": [100, 100, 100, 100, 100, 100, 50, 50, 50, 100],
            "Fist": [15, 35, 0, 0, 0, 0, 50, 50, 50, 30],
            "One": [15, 35, 100, 0, 0, 0, 50, 50, 50, 30],
            "Two": [15, 35, 100, 100, 0, 0, 50, 50, 50, 30],
            "Three": [15, 35, 100, 100, 100, 0, 50, 50, 50, 30],
            "OK": [35, 35, 35, 100, 100, 100, 50, 50, 50, 35],
            "Thumbs Up": [100, 100, 0, 0, 0, 0, 50, 50, 50, 100],
        },
    ),
    "L20": GuiHandConfig(
        joint_names=[
            "Thumb abduction",
            "Thumb yaw",
            "Thumb root",
            "Thumb tip",
            "Index abduction",
            "Index root",
            "Index tip",
            "Middle abduction",
            "Middle root",
            "Middle tip",
            "Ring abduction",
            "Ring root",
            "Ring tip",
            "Pinky abduction",
            "Pinky root",
            "Pinky tip",
        ],
        init_pos=[
            100,
            100,
            100,
            100,
            50,
            100,
            100,
            50,
            100,
            100,
            50,
            100,
            100,
            50,
            100,
            100,
        ],
        preset_actions={
            "Open": [100, 100, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100],
            "Fist": [70, 62, 31, 52, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "Fist Release": [70, 62, 60, 52, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "One": [45, 40, 70, 25, 50, 100, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "Two": [48, 21, 53, 40, 50, 100, 100, 50, 100, 100, 50, 0, 0, 50, 0, 0],
            "Three": [38, 12, 53, 40, 50, 100, 100, 50, 100, 100, 50, 100, 100, 50, 0, 0],
            "OK": [61, 45, 76, 38, 52, 33, 33, 50, 100, 100, 50, 100, 100, 50, 100, 100],
            "Thumb to middle": [61, 31, 75, 38, 52, 100, 100, 56, 27, 28, 50, 100, 100, 50, 100, 100],
            "Thumb to ring": [53, 20, 73, 38, 52, 100, 100, 56, 100, 100, 50, 28, 31, 50, 100, 100],
            "Thumbs Up": [100, 100, 100, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "All right": [100, 100, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100],
            "All left": [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100],
            "Spread": [100, 100, 100, 100, 0, 100, 100, 17, 100, 100, 71, 100, 100, 100, 100, 100],
            "Squeeze": [100, 100, 100, 100, 88, 100, 100, 48, 100, 100, 42, 100, 100, 25, 100, 100],
            "Grab wide": [100, 45, 100, 100, 0, 66, 12, 17, 60, 21, 75, 52, 27, 100, 47, 29],
            "Tip bend": [100, 55, 100, 0, 50, 100, 0, 50, 100, 0, 50, 100, 0, 50, 100, 0],
            "Index Left": [45, 40, 70, 25, 100, 100, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "Index forward": [45, 40, 70, 25, 50, 64, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "Index right": [45, 40, 70, 25, 0, 100, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "Thumb straight": [26, 48, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100],
        },
        cycle_loop_actions=["Index Left", "Index forward", "Index right"],
        cycle_loop_repeats=2,
    ),
    "L25": GuiHandConfig(
        joint_names=[
            "Thumb abduction",
            "Thumb yaw",
            "Thumb root",
            "Thumb tip",
            "Index abduction",
            "Index root",
            "Index tip",
            "Middle abduction",
            "Middle root",
            "Middle tip",
            "Ring abduction",
            "Ring root",
            "Ring tip",
            "Pinky abduction",
            "Pinky root",
            "Pinky tip",
        ],
        init_pos=[100, 100, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100],
        preset_actions={
            "Open": [100, 100, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100, 50, 100, 100],
            "Fist": [45, 35, 10, 10, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "One": [45, 35, 10, 10, 50, 100, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0],
            "Two": [45, 35, 10, 10, 50, 100, 100, 50, 100, 100, 50, 0, 0, 50, 0, 0],
            "Three": [45, 35, 10, 10, 50, 100, 100, 50, 100, 100, 50, 100, 100, 50, 0, 0],
            "OK": [40, 25, 45, 35, 50, 45, 35, 50, 100, 100, 50, 100, 100, 50, 100, 100],
            "Thumbs Up": [100, 100, 100, 100, 50, 0, 0, 50, 0, 0, 50, 0, 0, 50, 0, 0],
        },
    ),
}


def _fallback_config(model: str) -> GuiHandConfig:
    spec = get_hand_model_spec(model)
    default = DEFAULT_HAND_CONFIGS.get(spec.name)
    if default is not None:
        return default
    init_pos = [50] * spec.joint_count
    return GuiHandConfig(
        joint_names=[name.replace("_", " ").title() for name in spec.joint_names],
        init_pos=init_pos,
        preset_actions={"Open": [100] * spec.joint_count, "Neutral": init_pos},
    )


def get_gui_hand_config(model: str) -> GuiHandConfig:
    """Load hand GUI presets from the Python SDK when available."""

    spec = get_hand_model_spec(model)
    try:
        from realhand.gui_presets import HAND_CONFIGS

        config = HAND_CONFIGS.get(spec.name)
        if config and len(config.joint_names) == spec.joint_count:
            return GuiHandConfig(
                joint_names=list(config.joint_names),
                init_pos=[int(value) for value in config.init_pos],
                preset_actions={
                    name: [int(value) for value in values]
                    for name, values in config.preset_actions.items()
                    if len(values) == spec.joint_count
                },
                cycle_loop_actions=list(getattr(config, "cycle_loop_actions", [])),
                cycle_loop_repeats=int(getattr(config, "cycle_loop_repeats", 0)),
            )
    except Exception:
        pass
    return _fallback_config(model)
