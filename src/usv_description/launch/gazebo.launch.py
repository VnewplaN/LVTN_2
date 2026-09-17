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
    # Link to package
    package_dir = get_package_share_directory("usv_description")
    gazebo_package_dir = get_package_share_directory("ros_gz_sim")

    # Link to model (use declare argument to allow flexible input argument)
    model_path = os.path.join(package_dir, "urdf", "usv.urdf.xacro")
    usv_model = DeclareLaunchArgument(
        "usv_model",
        default_value=model_path,
        description="Absolute path to robot urdf file",
    )
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.2817")
    spawn_roll_arg = DeclareLaunchArgument("spawn_roll", default_value="0.0")
    spawn_pitch_arg = DeclareLaunchArgument("spawn_pitch", default_value="0.0")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_roll = LaunchConfiguration("spawn_roll")
    spawn_pitch = LaunchConfiguration("spawn_pitch")

    # Declare world path
    world_path = os.path.join(
        package_dir,
        "urdf",
        "environment.sdf",
    )

    # Resource path (for gazebo to find meshes)
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(package_dir).parent.resolve())],
    )

    # Convert xacro to standard URDF XML
    usv_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("usv_model")]),
        value_type=str,
    )

    # Run state publisher executable (TF transform illustrate)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": usv_description,
                "use_sim_time": True,
            }
        ],
    )

    # Initialize Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_package_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": ["-r ", world_path]}.items(),
    )

    # Spawn USV
    gz_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "industrial_sludge_pond",
            "-topic",
            "robot_description",
            "-name",
            "usv",
            "-x",
            "-25.0",
            "-y",
            "-40.0",
            # Initial guess only; graded buoyancy determines
            # the final hydrostatic draft.
            "-z",
            spawn_z,
            "-R",
            spawn_roll,
            "-P",
            spawn_pitch,
            "-Y",
            "0.0",
        ],
    )

   # Bridge simulation time
    gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

        "/world/industrial_sludge_pond/wrench"
        "@ros_gz_interfaces/msg/EntityWrench]"
        "gz.msgs.EntityWrench",

        "/model/usv/odometry"
        "@nav_msgs/msg/Odometry["
        "gz.msgs.Odometry",

        "/model/usv/pose"
        "@tf2_msgs/msg/TFMessage["
        "gz.msgs.Pose_V",

        "/model/usv/joint_state"
        "@sensor_msgs/msg/JointState["
        "gz.msgs.Model",

        "/world/industrial_sludge_pond/set_pose"
        "@ros_gz_interfaces/srv/SetEntityPose",
    ],

    remappings=[
        ("/model/usv/odometry", "/odom"),
        ("/model/usv/pose", "/tf"),
        ("/model/usv/joint_state", "/joint_states"),
    ],
    )
    # Separate nodes allow each lidar scan to use its own URDF frame.
    lidar_front_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/lidar/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[{"override_frame_id": "lidar_front"}],
    )

    lidar_back_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/lidar/back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        parameters=[{"override_frame_id": "lidar_back"}],
    )

    return LaunchDescription(
        [
            gazebo_resource_path,
            usv_model,
            spawn_z_arg,
            spawn_roll_arg,
            spawn_pitch_arg,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_robot,
            gz_ros2_bridge,
            lidar_front_bridge,
            lidar_back_bridge,
        ]
    )
