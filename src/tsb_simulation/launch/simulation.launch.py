import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Locate installed packages without hard coding paths
    simulation_share = get_package_share_directory('tsb_simulation')
    gazebo_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(simulation_share, "worlds", "control_world.sdf")
    models_path = os.path.join(simulation_share, "models")
    bridge_config = os.path.join(simulation_share, "config", "bridge.yaml")

    # Search model directory first, preserving existing resource paths
    existing_resource_paths = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_paths = models_path

    if existing_resource_paths:
        resource_paths += os.pathsep + existing_resource_paths

    configure_resources = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_paths
    )

    # Start Gazebo server and GUI, with the simulation playing
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": f"-r {world_path}",
            "on_exit_shutdown": "true",
        }.items()
    )  

    # Connect topics declared in the bridge.yaml
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulation_bridge",
        output="screen",
        parameters=[
            {"config_file": bridge_config},
        ],
    )

    return LaunchDescription([
        configure_resources,
        gazebo,
        bridge,
    ])  
