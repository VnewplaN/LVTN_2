import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    #link to package
    package_dir = get_package_share_directory("usv_description")
    #link to model (use declare argument to allow flexible input argument)
    model_path = os.path.join(package_dir, "urdf", "usv.urdf.xacro")
    usv_model = DeclareLaunchArgument("usv_model", default_value=model_path,
                                      description="Absolute path to robot urdf file")
    #convert xacro to standard urdf xml
    usv_description = ParameterValue(Command(["xacro ", LaunchConfiguration("usv_model")]),
                                     value_type=str)
    #run state publisher executable (link illustate)
    robot_state_publisher_node = Node(
                                      package="robot_state_publisher",
                                      executable="robot_state_publisher",
                                      parameters=[{
                                          "robot_description": usv_description,
                                          "use_sim_time": True,
                                      }]
                                      )
    #rviz run
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(package_dir, "rviz", "display.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([
                            usv_model,
                            robot_state_publisher_node,
                            rviz_node])
