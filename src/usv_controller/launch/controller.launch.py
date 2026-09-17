import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("usv_controller")
    config_path = os.path.join(package_dir, "config", "config.yaml")
    keyboard_control = LaunchConfiguration("keyboard_control")

    common_parameters = [
        config_path,
        {
            "use_sim_time": True,
            "keyboard_control_flag": ParameterValue(
                keyboard_control,
                value_type=bool,
            ),
        },
    ]

    # ros2 launch gives a normal child process a pipe on stdin. Open a real
    # terminal so read(STDIN_FILENO, ...) can receive keyboard input.
    keyboard_controller = Node(
        package="usv_controller",
        executable="thruster_controller",
        name="thruster_controller",
        output="screen",
        parameters=common_parameters,
        prefix="gnome-terminal --title=USV-Keyboard-Control --",
        condition=IfCondition(keyboard_control),
    )

    automatic_controller = Node(
        package="usv_controller",
        executable="thruster_controller",
        name="thruster_controller",
        output="screen",
        parameters=common_parameters,
        condition=UnlessCondition(keyboard_control),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "keyboard_control",
                default_value="false",
                description="Open a terminal for keyboard input",
            ),
            keyboard_controller,
            automatic_controller,
        ]
    )
