
cmake_minimum_required(VERSION 3.8)
project(moveit2_ompl_demo)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ompl_planner_node src/ompl_planner_node.cpp)
ament_target_dependencies(ompl_planner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS ompl_planner_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
