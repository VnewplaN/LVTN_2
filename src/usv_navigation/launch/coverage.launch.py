import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("usv_navigation")
    config_path = os.path.join(package_dir, "config", "config.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    track_spacing = LaunchConfiguration("track_spacing")
    map_margin = LaunchConfiguration("map_margin")
    path_speed = LaunchConfiguration("path_speed")

    coverage_planner = Node(
        package="usv_navigation",
        executable="coverage_planner",
        name="coverage_planner",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "track_spacing": ParameterValue(track_spacing, value_type=float),
                "map_margin": ParameterValue(map_margin, value_type=float),
            },
        ],
    )

    trajectory_node = Node(
        package="usv_navigation",
        executable="trajectory_node",
        name="trajectory_node",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "path_speed": ParameterValue(path_speed, value_type=float),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("track_spacing", default_value="0.25"),
            DeclareLaunchArgument("map_margin", default_value="3.0"),
            DeclareLaunchArgument("path_speed", default_value="0.5"),
            coverage_planner,
            trajectory_node,
        ]
    )
