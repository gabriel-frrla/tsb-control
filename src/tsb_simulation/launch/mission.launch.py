import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("tsb_simulation")
    control_share = get_package_share_directory("tsb_control")

    # Reuse the simulation, bridge, sensor processing, and PID launch.
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_share, "launch", "control.launch.py")
        )
    )

    # Generate speed and heading references from the configured waypoints.
    path_following = Node(
        package="tsb_control",
        executable="path_following_node",
        name="path_following_node",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "path_following.yaml"),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        control,
        path_following,
    ])
