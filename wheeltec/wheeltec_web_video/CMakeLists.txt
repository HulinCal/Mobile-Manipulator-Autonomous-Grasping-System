cmake_minimum_required(VERSION 3.8)
project(web_video_server)

# Jazzy uses C++17 by default
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(async_web_server_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED IMPORTED_TARGET)
pkg_check_modules(avformat libavformat REQUIRED IMPORTED_TARGET)
pkg_check_modules(avutil libavutil REQUIRED IMPORTED_TARGET)
pkg_check_modules(swscale libswscale REQUIRED IMPORTED_TARGET)

###################################################
## Declare things to be passed to other projects ##
###################################################

###########
## Build ##
###########

include_directories(include
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/web_video_server.cpp
  src/image_streamer.cpp
  src/libav_streamer.cpp
  src/vp8_streamer.cpp
  src/h264_streamer.cpp
  src/vp9_streamer.cpp
  src/multipart_stream.cpp
  src/ros_compressed_streamer.cpp
  src/jpeg_streamers.cpp
  src/png_streamers.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  async_web_server_cpp cv_bridge image_transport rclcpp sensor_msgs)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  PkgConfig::avcodec
  PkgConfig::avformat
  PkgConfig::avutil
  PkgConfig::swscale
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_package()
