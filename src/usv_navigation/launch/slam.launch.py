import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory("usv_navigation")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")

    scan_merger = Node(
        package="usv_navigation",
        executable="laser_scan_merger",
        name="laser_scan_merger",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": os.path.join(
                navigation_share, "config", "slam.yaml"
            ),
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription([scan_merger, slam])
