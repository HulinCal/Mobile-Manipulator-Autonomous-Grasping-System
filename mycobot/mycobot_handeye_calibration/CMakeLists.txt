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

ament_package()
