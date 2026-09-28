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
from launch.conditions import IfCondition


def generate_launch_description():
    # Link to package
    package_dir = get_package_share_directory("usv_navigation")
    rviz_pkg_dir = get_package_share_directory("usv_description")
    config_path = os.path.join(package_dir, "config", "config.yaml")
    slam_config_path = os.path.join(package_dir, "config", "slam.yaml")
    
    map_save_path = os.path.expanduser(
        "~/LVTN_2/src/usv_navigation/maps/usv_map"
    )
    map_yaml_path = map_save_path + ".yaml"
    run_map_auto_default = (
        "false" if os.path.exists(map_yaml_path) else "true"
    )
    run_map_auto = LaunchConfiguration("run_map_auto")
    slam_toolbox_dir = get_package_share_directory(
    "slam_toolbox"
    )

    slam_toolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            slam_toolbox_dir,
            "launch",
            "online_async_launch.py",
        )
    ),
    launch_arguments={
        "slam_params_file": slam_config_path,
        "use_sim_time": "true",
    }.items(),
    )   
    lidar_merge = Node(
        package="usv_navigation",
        executable="laser_scan_merger",
        name="laser_scan_merger",
         output="screen",
        parameters=  [config_path],
    )

    map_auto = Node(
        package="usv_navigation",
        executable="map_auto",
        name="map_auto_mapping",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": True,
                "map_save_path": map_save_path,
            },
        ],
        condition=IfCondition(run_map_auto),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(rviz_pkg_dir, "rviz", "display.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_map_auto",
                default_value=run_map_auto_default,
                description=(
                    "Run map_auto; defaults to false when usv_map.yaml exists"
                ),
            ),
            lidar_merge,
            map_auto,
            slam_toolbox,
            rviz_node,
        ]
    )
