cmake_minimum_required(VERSION 3.8)
project(wheeltec_nav2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Jazzy uses C++17 by default
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)

# 初始化定位 Navigator 插件依赖
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 定位异常监控节点依赖
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 自定义行（action）：初始化定位
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/InitializeLocalization.action"
)

# 初始化定位 Navigator 插件
add_library(initialize_localization_navigator SHARED
  src/initialize_localization_navigator.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(initialize_localization_navigator
  nav2_core
  nav2_behavior_tree
  nav2_util
  ament_index_cpp
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
)

target_link_libraries(initialize_localization_navigator
  ${cpp_typesupport_target}
)

# 定位异常监控节点：协方差 + 激光/地图一致性校验，异常时自动触发初始化定位
add_executable(localization_monitor
  src/localization_monitor.cpp
)

ament_target_dependencies(localization_monitor
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(localization_monitor
  ${cpp_typesupport_target}
)

install(
  TARGETS initialize_localization_navigator localization_monitor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 把 Navigator 插件注册到 nav2_core 插件分类
pluginlib_export_plugin_description_file(nav2_core navigator_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter (no copyright headers in source files)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
