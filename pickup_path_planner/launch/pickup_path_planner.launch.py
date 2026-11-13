from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("pickup_path_planner"))
    parameter_file = package_share / "config" / "planner.yaml"

    return LaunchDescription(
        [
            Node(
                package="pickup_path_planner",
                executable="pickup_path_planner_node",
                name="pickup_path_planner",
                output="screen",
                parameters=[str(parameter_file)],
            )
        ]
    )
