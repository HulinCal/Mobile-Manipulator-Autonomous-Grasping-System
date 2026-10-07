# ROS2 Humble → Jazzy 转换记录

本文档记录了将 wheeltec_ws 工作空间从 ROS2 Humble 迁移到 Jazzy 版本的所有修改。

---

## 1. wheeltec_robot_urdf

### CMakeLists.txt
- C++ 标准从 C++14 升级到 C++17
- 添加 `ament_lint_auto_set_lint_dependencies()`、`ament_lint_auto_find_test_dependencies()` 测试配置
- 添加 `BUILD_TESTING` 块以启用 lint 测试

### package.xml
- `buildtool_depend` 从 `ament_cmake` 改为 `ament_cmake`（保持），添加 lint 依赖
- 添加 `ament_lint_auto` 和 `ament_lint_common` 测试依赖

---

## 2. wheeltec_joy

### src/wheeltec_joy_control.cpp
- 修复参数声明逻辑：为所有 `declare_parameter` 调用提供默认值
- 例如：`this->declare_parameter("serial_baud_rate");` → `this->declare_parameter("serial_baud_rate", 115200);`
- 修复参数获取时的类型推断

### CMakeLists.txt
- C++ 标准升级到 C++17
- 调整依赖声明

### package.xml
- 更新依赖声明以匹配 Jazzy

---

## 3. wheeltec_robot_nav2

### CMakeLists.txt
- C++ 标准升级到 C++17
- 安装路径调整

### package.xml
- 更新依赖声明以匹配 Jazzy

---

## 4. wheeltec_robot_keyboard

### setup.cfg
- 将 `script-dir` 改为 `script_dir`（避免弃用警告）
- 将 `install-scripts` 改为 `install_scripts`（避免弃用警告）

```ini
[develop]
script_dir=$base/lib/wheeltec_robot_keyboard
[install]
install_scripts=$base/lib/wheeltec_robot_keyboard
```

---

## 5. wheeltec_robot_rtab

### CMakeLists.txt
- C++ 标准升级到 C++17
- 依赖调整

### package.xml
- 更新依赖声明以匹配 Jazzy

---

## 6. wheeltec_rrt_msg

### CMakeLists.txt
- 添加 `ament_lint_auto_set_lint_dependencies()` 测试配置
- 添加 `BUILD_TESTING` 块以启用 lint 测试
- C++ 标准升级到 C++17

### package.xml
- 添加 `ament_lint_auto` 和 `ament_lint_common` 测试依赖

---

## 7. usb_cam (usb_cam-ros2)

### include/usb_cam/usb_cam.hpp
- 添加 `#include <libavutil/imgutils.h>` 头文件（用于 `av_image_get_buffer_size`、`av_image_fill_arrays`、`av_image_copy_to_buffer`）
- 成员变量 `AVCodec * avcodec_` 改为 `const AVCodec * avcodec_`（FFmpeg 4.0+ 中 `avcodec_find_decoder` 返回 const）

### src/usb_cam.cpp
- `init_mjpeg_decoder` 函数：
  - 移除 `avcodec_register_all()` 调用（FFmpeg 4.0+ 不再需要）
  - 移除 `const_cast<AVCodec *>`（成员已是 const）
  - 使用 `av_image_get_buffer_size` 替代 `avpicture_alloc`
  - 使用 `av_image_fill_arrays` 替代 `avpicture_fill`
- `mjpeg2rgb` 函数：
  - 使用 `av_packet_alloc()` / `av_packet_free()` 替代栈上 `AVPacket` 和 `av_init_packet`
  - 使用 `avcodec_send_packet` + `avcodec_receive_frame` 替代弃用的 `avcodec_decode_video2`
  - 使用 `av_image_get_buffer_size` 替代 `avpicture_get_size`
  - 使用 `av_image_copy_to_buffer` 替代 `avpicture_layout`

### CMakeLists.txt
- C++ 标准升级到 C++17
- 添加 `avutil` 的 `pkg_check_modules` 依赖
- FFmpeg 库使用 `IMPORTED_TARGET` 方式链接：`PkgConfig::avcodec`、`PkgConfig::swscale`、`PkgConfig::avutil`

---

## 8. wheeltec_web_video (web_video_server)

### CMakeLists.txt
- C++ 标准从 C++14 升级到 C++17
- 添加 `-Wall -Wextra -Wpedantic` 编译选项
- FFmpeg 库使用 `IMPORTED_TARGET` 方式链接：`PkgConfig::avcodec`、`PkgConfig::avformat`、`PkgConfig::avutil`、`PkgConfig::swscale`
- 移除 `include_directories` 中的 FFmpeg include 路径（由 IMPORTED_TARGET 自动处理）

### include/web_video_server/libav_streamer.h
- `AVCodec* codec_` 改为 `const AVCodec* codec_`（FFmpeg 4.0+ 返回 const）
- `AVOutputFormat* output_format_` 改为 `const AVOutputFormat* output_format_`

### include/web_video_server/web_video_server.h
- `#include <cv_bridge/cv_bridge.h>` 改为 `#include <cv_bridge/cv_bridge.hpp>`（Jazzy 中 .h 弃用）

### src/image_streamer.cpp
- `#include <cv_bridge/cv_bridge.h>` 改为 `#include <cv_bridge/cv_bridge.hpp>`
- `rclcpp::Duration(max_age)` 改为 `rclcpp::Duration::from_seconds(max_age)`（Jazzy 中 Duration 不再接受 double 参数）

### src/libav_streamer.cpp
- 移除 `av_lockmgr_register` 和 `ffmpeg_boost_mutex_lock_manager`（FFmpeg 4.0+ 已删除该 API）
- 移除 `av_register_all()` 调用（FFmpeg 4.0+ 不再需要）
- `video_stream_->codec` 改为 `avcodec_alloc_context3(codec_)`（codec 字段已弃用）
- 移除 `avcodec_get_context_defaults3` 调用（已弃用）
- 在 `avcodec_open2` 后才调用 `avformat_new_stream`，并使用 `avcodec_parameters_from_context` 同步参数
- `output_format_->flags` 赋值使用 `const_cast<AVOutputFormat*>(output_format_)`（因 output_format_ 现在是 const）
- 析构函数中 `avcodec_close` 改为 `avcodec_free_context`
- `sendImage` 中编码部分：
  - 移除所有 `#if LIBAVCODEC_VERSION_INT` 版本条件分支
  - 使用 `av_packet_alloc()` / `av_packet_free()` 替代栈上 `AVPacket` 和 `av_init_packet`
  - 使用 `avcodec_send_frame` + `avcodec_receive_packet` 替代 `avcodec_encode_video2` 等弃用 API
  - 指针访问改为 `pkt->` 形式（因 pkt 现在是指针）

### src/ros_compressed_streamer.cpp
- `rclcpp::Duration(max_age)` 改为 `rclcpp::Duration::from_seconds(max_age)`

### src/web_video_server.cpp
- 添加 `#include <boost/bind/bind.hpp>`
- 所有 `_1`、`_2`、`_3`、`_4` 占位符改为 `boost::placeholders::_1` 等（Boost 1.73+ 需要显式命名空间）

### 系统依赖
- 安装 `ros-jazzy-async-web-server-cpp` 系统包（Jazzy 源中提供）

---

## 9. wheeltec_robot_rrt (wheeltec_robot_rrt2)

### CMakeLists.txt
- `cmake_minimum_required` 版本从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17
- OpenCV 依赖移除版本约束（`find_package(OpenCV 4.2.0 REQUIRED)` → `find_package(OpenCV REQUIRED)`）
- `behaviortree_cpp_v3` 依赖改为 `behaviortree_cpp`（Jazzy 中 nav2_behavior_tree 使用 v4）
- 为 `utils` 库添加 `nav_msgs`、`visualization_msgs`、`geometry_msgs` 依赖
- 为 `wait_for_fin` 添加 `visualization_msgs` 依赖

### package.xml
- 补全所有缺失的依赖声明：`rclcpp_action`、`filters`、`wheeltec_rrt_msg`、`tf2_ros`、`tf2_geometry_msgs`、`visualization_msgs`、`nav2_msgs`、`std_msgs`、`laser_geometry`、`sensor_msgs`、`cv_bridge`、`behaviortree_cpp`、`nav2_behavior_tree`、`nav2_util`
- `behaviortree_cpp_v3` 改为 `behaviortree_cpp`（v4）

### src/robot_picker.cpp
- 头文件 `behaviortree_cpp_v3/utils/shared_library.h` 改为 `behaviortree_cpp/utils/shared_library.h`
- `BehaviorTreeEngine` 构造增加第二个参数 `node`（Jazzy 中需要 `rclcpp::Node::SharedPtr`）
- `bt_->haltAllActions(tree_.rootNode())` 改为 `bt_->haltAllActions(tree_)`（Jazzy 中签名变更）
- `switch` 语句增加 `SUCCEEDED` 分支以避免 `-Wswitch` 警告

### src/bt_plugins/*.cpp
- `find_coloured_box.cpp`、`approach_coloured_box.cpp`、`pick_coloured_box.cpp`：
  - 头文件 `behaviortree_cpp_v3/bt_factory.h` 改为 `behaviortree_cpp/bt_factory.h`

### src/rrt_exploration/include/utils.h
- `visualization_msgs/msg/marker.h` 改为 `visualization_msgs/msg/marker.hpp`（Jazzy 中 .h 弃用）

### 系统依赖
- 安装 `ros-jazzy-behaviortree-cpp-v3`、`ros-jazzy-nav2-behavior-tree`（含 `behaviortree_cpp` v4）

---

## 10. wheeltec_robot_slam (4 个子包)

### openslam_gmapping
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

### slam_gmapping
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17
- `include/slam_gmapping/slam_gmapping.h` 中 `tf2_geometry_msgs/tf2_geometry_msgs.h` 改为 `tf2_geometry_msgs/tf2_geometry_msgs.hpp`（Jazzy 中 .h 弃用）

### wheeltec_cartographer
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

### wheeltec_slam_toolbox
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

---

## 11. wheeltec_lidar_ros2 (6 个子包)

### rplidar_A1 (lidar_ros2)
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17
- `sdk/src/hal/event.h` 中 `enum` 改为 `enum : unsigned long`，修复 C++17 下枚举到 unsigned long 的窄化转换错误
- `sdk/src/arch/linux/net_socket.cpp` 中 `ans<=0` 改为 `ans == nullptr`，修复指针与整数比较错误

### rplidar_ros-ros2
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17
- 移除硬编码的 `set(CMAKE_CXX_FLAGS "-std=c++11 ...")` 以使用项目级 C++17 设置
- `sdk/src/hal/event.h` 中 `enum` 改为 `enum : unsigned long`，修复 C++17 下窄化转换错误
- `sdk/src/arch/linux/net_socket.cpp` 中 `ans<=0` 改为 `ans == nullptr`，修复指针与整数比较错误
- `src/rplidar_scan_publisher.cpp` 中所有 `declare_parameter("name")` 改为带默认值的形式，如 `declare_parameter<std::string>("channel_type", "serial")`，适应 Jazzy 要求

### LDlidar/ldlidar06 (ldlidar_stl_ros2)
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

### LDlidar/ldlidar14 (ldlidar_sl_ros2)
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

### LSlidar/lslidar_msgs
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

### LSlidar/lslidar_driver
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

### 系统依赖
- 安装 `libpcap-dev`（lslidar_driver 依赖 pcap.h）

---

## 12. wheeltec_robot_msg

该包已预先配置为 Jazzy 兼容（C++17、cmake 3.8），无需修改。

---

## 13. simple_follower_ros2

### setup.cfg
- `script-dir` 改为 `script_dir`（Python setuptools 新规范）
- `install-scripts` 改为 `install_scripts`

---

## 14. 简单 Python 包（beamer、keyboard_control_emer、manual_drive、laser_scan_reader、kul_tabloo_launch、quiz_pi_con、quiz_status）

这些包已预先配置为 Jazzy 兼容（`script_dir`/`install_scripts` 格式），无需修改。所有包均成功编译。

---

## 15. ar_track_alvar

### CMakeLists.txt
- `cmake_minimum_required` 从 3.5 升级到 3.8
- 添加 C++17 标准设置
- 移除 `find_package(perception_pcl REQUIRED)` 和 `find_package(tinyxml_vendor REQUIRED)` 依赖（Jazzy 中已弃用）
- 使用 `pkg_check_modules(TINYXML REQUIRED tinyxml)` 替代 `tinyxml_vendor`
- 从依赖列表中移除 `perception_pcl`
- `target_link_libraries` 中所有 `${TinyXML_LIBRARIES}` 改为 `${TINYXML_LIBRARIES}`
- `ament_export_dependencies` 中移除 `perception_pcl`

### package.xml
- 移除 `<depend>perception_pcl</depend>`
- 移除 `<depend>rosbag2_bag_v2_plugins</depend>`

### 源代码修改
- `include/ar_track_alvar/Camera.h`: `resource_retriever/retriever.h` 改为 `retriever.hpp`
- `src/kinect_filtering.cpp`: `boost::make_shared<pcl::PointIndices>()` 改为 `pcl::make_shared<pcl::PointIndices>()`
- `nodes/IndividualMarkers.cpp`、`FindMarkerBundles.cpp`: 移除 `using boost::make_shared`
- `nodes/IndividualMarkers.cpp`、`FindMarkerBundles.cpp`: `BOOST_FOREACH` 改为 C++11 range-based for 循环
- 所有 nodes/*.cpp: `cv_bridge/cv_bridge.h` 改为 `cv_bridge/cv_bridge.hpp`
- 所有 nodes/*.cpp: `tf2_geometry_msgs/tf2_geometry_msgs.h` 改为 `tf2_geometry_msgs/tf2_geometry_msgs.hpp`
- 所有 nodes/*.cpp: `rclcpp::Duration(1.0)` 改为 `rclcpp::Duration::from_seconds(1.0)`（Jazzy 中 Duration 构造函数不再接受 double）

## 16. ar_track_alvar_msgs

### CMakeLists.txt
- `cmake_minimum_required` 从 3.5 升级到 3.8
- C++ 标准从 C++14 升级到 C++17

## 17. perception_pcl-foxy-devel (pcl_ros, pcl_conversions)

- 添加 `COLCON_IGNORE` 文件，跳过这些旧包的构建
- `pcl_ros/include/pcl_ros/impl/transforms.hpp` 和 `pcl_ros/src/transforms.cpp` 中 `tf2_geometry_msgs/tf2_geometry_msgs.h` 改为 `.hpp`（作为参考，但实际不构建）
- 使用 Jazzy 系统提供的 `ros-jazzy-pcl-ros` 和 `ros-jazzy-pcl-conversions` 替代

## 18. navigation2-galactic

- 添加 `COLCON_IGNORE` 文件跳过构建
- 使用 Jazzy 系统提供的 `ros-jazzy-navigation2` 替代

### 系统依赖
- 安装 `libtinyxml-dev`（替代 `tinyxml_vendor`）
- 安装 `ros-jazzy-nav2-bringup` 等 nav2 系统包
