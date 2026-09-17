cmake_minimum_required(VERSION 3.8)

project(usv_navigation)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(
  laser_scan_merger
  src/laser_scan_merger.cpp
)

ament_target_dependencies(
  laser_scan_merger
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(
  mapping_route
  src/mapping_route.cpp
)

ament_target_dependencies(
  mapping_route
  rclcpp
  std_msgs
)

add_executable(
  coverage_planner
  src/coverage_planner.cpp
)

ament_target_dependencies(
  coverage_planner
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(
  trajectory_node
  src/trajectory_node.cpp
)

add_executable(
  coverage_playback
  src/coverage_playback.cpp
)

ament_target_dependencies(
  coverage_playback
  rclcpp
  nav_msgs
  ros_gz_interfaces
)

ament_target_dependencies(
  trajectory_node
  rclcpp
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    laser_scan_merger
    mapping_route
    coverage_planner
    trajectory_node
    coverage_playback
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY
    launch
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


ament_package()
