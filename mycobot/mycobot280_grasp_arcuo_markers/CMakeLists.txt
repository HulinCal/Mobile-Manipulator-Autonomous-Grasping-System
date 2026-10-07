cmake_minimum_required(VERSION 3.8)
project(mycobot280_grasp_arcuo_markers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(detect_markers_node src/detect_markers_node.cpp)
ament_target_dependencies(detect_markers_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(grasp_node src/grasp_node.cpp)
ament_target_dependencies(grasp_node
  rclcpp
  tf2
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
)
install(TARGETS detect_markers_node grasp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/camera_link_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_link_tf_publisher
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
