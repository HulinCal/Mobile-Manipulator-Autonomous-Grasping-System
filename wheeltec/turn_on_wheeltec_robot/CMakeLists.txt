cmake_minimum_required(VERSION 3.8)
project(turn_on_wheeltec_robot)

# Jazzy uses C++17 by default
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(wheeltec_robot_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter (no copyright headers in source files)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

add_executable(wheeltec_robot_node
  src/wheeltec_robot.cpp
  src/Quaternion_Solution.cpp
)
target_include_directories(wheeltec_robot_node PRIVATE include)
ament_target_dependencies(wheeltec_robot_node
  tf2_ros
  tf2
  tf2_geometry_msgs
  rclcpp
  std_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  ackermann_msgs
  wheeltec_robot_msg
)

add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

install(PROGRAMS scripts/cmd_vel_to_ackermann_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  wheeltec_robot_node
  parameter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
