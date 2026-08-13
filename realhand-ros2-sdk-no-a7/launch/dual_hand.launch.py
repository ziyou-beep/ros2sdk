from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_POLL_INTERVALS_JSON = (
    '{"angle": 0.03333333333333333, '
    '"force_sensor": 0.06666666666666667, '
    '"torque": 0.2, '
    '"speed": 0.5, '
    '"acceleration": 0.5, '
    '"temperature": 1.0, '
    '"current": 0.5, '
    '"fault": 1.0}'
)


def generate_launch_description():
    model = LaunchConfiguration("model")
    left_interface = LaunchConfiguration("left_interface")
    right_interface = LaunchConfiguration("right_interface")
    interface_type = LaunchConfiguration("interface_type")
    poll_on_start = LaunchConfiguration("poll_on_start")
    stream_on_start = LaunchConfiguration("stream_on_start")
    stream_queue_size = LaunchConfiguration("stream_queue_size")
    poll_intervals_json = LaunchConfiguration("poll_intervals_json")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="L6"),
            DeclareLaunchArgument("left_interface", default_value="can0"),
            DeclareLaunchArgument("right_interface", default_value="can1"),
            DeclareLaunchArgument("interface_type", default_value="socketcan"),
            DeclareLaunchArgument("poll_on_start", default_value="true"),
            DeclareLaunchArgument("stream_on_start", default_value="true"),
            DeclareLaunchArgument("stream_queue_size", default_value="300"),
            DeclareLaunchArgument(
                "poll_intervals_json", default_value=DEFAULT_POLL_INTERVALS_JSON
            ),
            Node(
                package="realhand_ros2",
                executable="realhand_hand_node",
                name="realhand_left_hand_node",
                output="screen",
                parameters=[
                    {
                        "model": model,
                        "side": "left",
                        "interface_name": left_interface,
                        "interface_type": interface_type,
                        "poll_on_start": ParameterValue(
                            poll_on_start, value_type=bool
                        ),
                        "stream_on_start": ParameterValue(
                            stream_on_start, value_type=bool
                        ),
                        "stream_queue_size": ParameterValue(
                            stream_queue_size, value_type=int
                        ),
                        "poll_intervals_json": ParameterValue(
                            poll_intervals_json, value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="realhand_ros2",
                executable="realhand_hand_node",
                name="realhand_right_hand_node",
                output="screen",
                parameters=[
                    {
                        "model": model,
                        "side": "right",
                        "interface_name": right_interface,
                        "interface_type": interface_type,
                        "poll_on_start": ParameterValue(
                            poll_on_start, value_type=bool
                        ),
                        "stream_on_start": ParameterValue(
                            stream_on_start, value_type=bool
                        ),
                        "stream_queue_size": ParameterValue(
                            stream_queue_size, value_type=int
                        ),
                        "poll_intervals_json": ParameterValue(
                            poll_intervals_json, value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
