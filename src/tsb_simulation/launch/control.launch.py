import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("tsb_simulation")
    control_share = get_package_share_directory("tsb_control")

    # Start Gazebo and the ROS–Gazebo bridge using our existing launch file.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                "launch",
                "simulation.launch.py",
            )
        )
    )

    # Validate sensor inputs and publish the measured state.
    sensor_processing = Node(
        package="tsb_control",
        executable="sensor_processing_node",
        name="sensor_processing_node",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "sensors.yaml"),
            {"use_sim_time": True},
        ],
    )

    # Read /state and /control/reference, then publish /cmd_vel.
    pid = Node(
        package="tsb_control",
        executable="pid_node",
        name="pid_node",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "pid.yaml"),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        simulation,
        sensor_processing,
        pid,
    ])
