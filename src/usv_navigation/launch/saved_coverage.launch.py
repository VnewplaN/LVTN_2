import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory("usv_navigation")
    map_file = LaunchConfiguration("map")
    track_spacing = LaunchConfiguration("track_spacing")
    show_rviz = LaunchConfiguration("show_rviz")

    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "yaml_filename": map_file,
            }
        ],
        remappings=[("/map", "/saved_map")],
    )

    configure_map_server = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate_map_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    planner = Node(
        package="usv_navigation",
        executable="coverage_planner",
        name="coverage_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "map_topic": "/saved_map",
                "track_spacing": ParameterValue(track_spacing, value_type=float),
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_coverage",
        output="screen",
        condition=IfCondition(show_rviz),
        arguments=["-d", os.path.join(share, "config", "coverage.rviz")],
        parameters=[{"use_sim_time": False}],
        remappings=[("/map", "/saved_map")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(share, "maps", "usv_map.yaml"),
                description="Saved occupancy-grid YAML file",
            ),
            DeclareLaunchArgument(
                "track_spacing",
                default_value="0.25",
                description="Distance between adjacent zigzag tracks in metres",
            ),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            map_server,
            configure_map_server,
            activate_map_server,
            planner,
            rviz,
        ]
    )
