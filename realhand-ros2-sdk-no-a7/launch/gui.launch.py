from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="realhand_ros2",
                executable="realhand_gui",
                name="realhand_gui",
                output="screen",
            )
        ]
    )
