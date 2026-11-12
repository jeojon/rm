from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("hik_camera")) / "config" / "camera.yaml"
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the HIKROBOT camera parameter YAML file",
            ),
            Node(
                package="hik_camera",
                executable="hik_camera_node",
                name="hik_camera",
                output="screen",
                emulate_tty=True,
                parameters=[config_file],
            ),
        ]
    )
