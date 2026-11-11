cmake_minimum_required(VERSION 3.8)
project(mycobot_handeye_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

# 关节滑块 GUI（拖动手柄控制机械臂位置）
install(PROGRAMS
  scripts/joint_slider_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
