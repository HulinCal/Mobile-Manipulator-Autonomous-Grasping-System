cmake_minimum_required(VERSION 3.8)
project(wheeltec_joy)

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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter (no copyright headers in source files)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(wheeltec_joy
  src/wheeltec_joy_control.cpp
)
ament_target_dependencies(wheeltec_joy
  rclcpp
  geometry_msgs
  sensor_msgs
)
install(TARGETS
  wheeltec_joy
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
