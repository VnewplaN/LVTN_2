cmake_minimum_required(VERSION 3.8)
project(usv_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(
  laser_scan_merger
  src/laser_scan_merger.cpp
)
add_executable(
  map_auto
  src/map_auto.cpp
)
add_executable(
  map_transform_loader
  src/map_transform_loader.cpp
)
add_executable(
  coverage_planner
  src/coverage_planner.cpp
)
add_executable(
  trajectory_node
  src/trajectory_node.cpp
)
ament_target_dependencies(
  laser_scan_merger
  rclcpp
  sensor_msgs
  tf2
)

ament_target_dependencies(
  map_transform_loader
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(
  map_auto
  geometry_msgs
  rclcpp
  sensor_msgs
  slam_toolbox
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(
  coverage_planner
  geometry_msgs
  nav_msgs
  rclcpp
)

ament_target_dependencies(
  trajectory_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    laser_scan_merger
    map_auto
    map_transform_loader
    coverage_planner
    trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
