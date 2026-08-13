from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model = LaunchConfiguration("model")
    side = LaunchConfiguration("side")
    interface_name = LaunchConfiguration("interface_name")
    interface_type = LaunchConfiguration("interface_type")
    world_frame = LaunchConfiguration("world_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="A7lite"),
            DeclareLaunchArgument("side", default_value="left"),
            DeclareLaunchArgument("interface_name", default_value=""),
            DeclareLaunchArgument("interface_type", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="urdf"),
            Node(
                package="realhand_ros2",
                executable="realhand_arm_node",
                name="realhand_arm_node",
                output="screen",
                parameters=[
                    {
                        "model": model,
                        "side": side,
                        "interface_name": interface_name,
                        "interface_type": interface_type,
                        "world_frame": world_frame,
                    }
                ],
            ),
        ]
    )
