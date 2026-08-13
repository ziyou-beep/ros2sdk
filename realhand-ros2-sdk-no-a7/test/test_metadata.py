from realhand_ros2.arm.model import get_arm_model_spec
from realhand_ros2.hand.model import get_hand_model_spec


def test_hand_model_metadata():
    assert get_hand_model_spec("L6").joint_count == 6
    assert get_hand_model_spec("L20lite").joint_count == 10
    assert get_hand_model_spec("L20").joint_count == 16


def test_arm_model_metadata():
    assert get_arm_model_spec("A7lite").joint_count == 7
    p7 = get_arm_model_spec("P7")
    assert p7.joint_count == 7
    assert p7.default_interface_type == "lbot"
    assert p7.default_interface_name == "192.168.10.21"
