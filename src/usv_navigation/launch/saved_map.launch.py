import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_package_dir = get_package_share_directory("usv_description")
    # map_auto writes into the source workspace so a newly generated map can
    # be reopened immediately without rebuilding the package.
    workspace_maps = os.path.expanduser("~/LVTN_2/src/usv_navigation/maps")
    default_map = os.path.join(workspace_maps, "usv_map.yaml")
    default_transform = os.path.join(workspace_maps, "usv_map.transform.yaml")

    map_yaml = LaunchConfiguration("map")
    transform_yaml = LaunchConfiguration("transform")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("transform", default_value=default_transform),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="usv_navigation",
                executable="map_transform_loader",
                name="map_transform_loader",
                output="screen",
                parameters=[
                    {"transform_file": transform_yaml, "use_sim_time": use_sim_time}
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_saved_map",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(rviz_package_dir, "rviz", "display.rviz"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
