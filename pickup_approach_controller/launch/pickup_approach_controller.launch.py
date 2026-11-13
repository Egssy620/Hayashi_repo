from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("pickup_approach_controller"))
    parameter_file = package_share / "config" / "controller.yaml"

    return LaunchDescription(
        [
            Node(
                package="pickup_approach_controller",
                executable="pickup_approach_controller_node",
                name="pickup_approach_controller",
                output="screen",
                parameters=[str(parameter_file)],
            )
        ]
    )
