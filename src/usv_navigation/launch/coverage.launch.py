import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("usv_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execute = LaunchConfiguration("execute")
    track_spacing = LaunchConfiguration("track_spacing")
    speed = LaunchConfiguration("speed")

    planner = Node(
        package="usv_navigation",
        executable="coverage_planner",
        name="coverage_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "track_spacing": ParameterValue(track_spacing, value_type=float),
            }
        ],
    )

    follower = Node(
        package="usv_navigation",
        executable="trajectory_node",
        name="trajectory_node",
        output="screen",
        condition=IfCondition(execute),
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "speed": ParameterValue(speed, value_type=float),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_coverage",
        output="screen",
        arguments=[
            "-d",
            os.path.join(share, "config", "coverage.rviz"),
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "execute",
                default_value="false",
                description="Follow the generated path and publish /trajectory_cmd",
            ),
            DeclareLaunchArgument(
                "track_spacing",
                default_value="0.25",
                description="Distance between adjacent zigzag tracks in metres",
            ),
            DeclareLaunchArgument("speed", default_value="1.0"),
            planner,
            follower,
            rviz,
        ]
    )
