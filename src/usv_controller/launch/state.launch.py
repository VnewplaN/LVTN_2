import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("usv_controller")
    #Get config file
    config_path = os.path.join(package_dir, "config", "config.yaml")
    #Create nodes
    state_reader_node = Node(
        package="usv_controller",
        executable="state_reader",
        name="state_reader",
        output="screen",
        parameters=[config_path, {"use_sim_time": True}],
    )

    return LaunchDescription(
        [
          state_reader_node
        ]
    )
