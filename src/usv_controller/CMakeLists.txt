cmake_minimum_required(VERSION 3.8)
project(usv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(state_reader src/state_reader.cpp)
ament_target_dependencies(state_reader 
                          rclcpp 
                          tf2 
                          nav_msgs 
                          tf2_geometry_msgs
                          std_msgs)

add_executable(thruster_controller src/thruster_controller.cpp)
ament_target_dependencies(thruster_controller
                          rclcpp 
                          tf2 
                          nav_msgs 
                          tf2_geometry_msgs
                          std_msgs
                          ros_gz_interfaces)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  state_reader
  thruster_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
