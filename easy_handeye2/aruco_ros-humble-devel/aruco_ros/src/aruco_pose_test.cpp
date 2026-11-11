// ArUco 位姿测试节点：
// 订阅相机彩色图像与内参，检测 ArUco 标记，并把当前图像 + 标记位姿（xyz/rpy）
// 叠加后发布到图像话题；只有检测到标记时才发布图像。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ArucoPoseTest : public rclcpp::Node
{
public:
  ArucoPoseTest()
  : Node("aruco_pose_test"), cam_info_received_(false)
  {
  }

  void setup()
  {
    // 参数：默认值与本项目的 Astra 相机 / 标定板（ID 582、5cm）一致
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/color/camera_info");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_color_optical_frame");
    marker_size_ = declare_parameter<double>("marker_size", 0.026);
    marker_id_ = declare_parameter<int>("marker_id", 2);
    image_is_rectified_ = declare_parameter<bool>("image_is_rectified", false);
    double min_marker_size = declare_parameter<double>("min_marker_size", 0.01);
    std::string detection_mode = declare_parameter<std::string>("detection_mode", "DM_VIDEO_FAST");

    if (detection_mode == "DM_FAST") {
      mDetector_.setDetectionMode(aruco::DM_FAST, static_cast<float>(min_marker_size));
    } else if (detection_mode == "DM_VIDEO_FAST") {
      mDetector_.setDetectionMode(aruco::DM_VIDEO_FAST, static_cast<float>(min_marker_size));
    } else {
      mDetector_.setDetectionMode(aruco::DM_NORMAL, static_cast<float>(min_marker_size));
    }

    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    image_sub_ = it_->subscribe(
      image_topic_, 1, &ArucoPoseTest::imageCallback, this);
    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ArucoPoseTest::camInfoCallback, this, std::placeholders::_1));

    image_pub_ = it_->advertise(this->get_name() + std::string("/image"), 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      this->get_name() + std::string("/pose"), 10);

    RCLCPP_INFO(
      get_logger(),
      "aruco_pose_test: image=%s camera_info=%s marker_id=%d marker_size=%.3f m",
      image_topic_.c_str(), camera_info_topic_.c_str(), marker_id_, marker_size_);
  }

private:
  void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
  {
    camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, image_is_rectified_);
    if (!cam_info_received_) {
      cam_info_received_ = true;
      RCLCPP_INFO(
        get_logger(), "收到相机内参 %ux%u，开始检测 ArUco 标记", msg->width, msg->height);
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!cam_info_received_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "尚未收到 camera_info，无法计算位姿");
      return;
    }
    if (image_pub_.getNumSubscribers() == 0 && pose_pub_->get_subscription_count() == 0) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 转换失败: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    markers_.clear();
    mDetector_.detect(image, markers_, camParam_, static_cast<float>(marker_size_), false);

    if (markers_.empty()) {
      RCLCPP_DEBUG(get_logger(), "未检测到 ArUco 标记，不发布图像");
      return;
    }

    std::vector<std::string> info_lines;
    info_lines.reserve(markers_.size() * 3);

    bool target_found = false;
    geometry_msgs::msg::PoseStamped target_pose;

    for (auto & marker : markers_) {
      marker.draw(image, cv::Scalar(0, 255, 0), 2);
      if (camParam_.isValid() && marker_size_ > 0.0) {
        aruco::CvDrawingUtils::draw3dAxis(image, marker, camParam_);
      }

      tf2::Transform transform = aruco_ros::arucoMarker2Tf2(marker);
      tf2::Quaternion q = transform.getRotation();
      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      const tf2::Vector3 & t = transform.getOrigin();
      char buf[256];
      std::snprintf(
        buf, sizeof(buf), "ID %d  dist %.3f m", marker.id, t.length());
      info_lines.push_back(buf);
      std::snprintf(
        buf, sizeof(buf), "  x %.3f  y %.3f  z %.3f m", t.x(), t.y(), t.z());
      info_lines.push_back(buf);
      std::snprintf(
        buf, sizeof(buf), "  rpy %.1f %.1f %.1f deg",
        roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
      info_lines.push_back(buf);

      if (!target_found && (marker_id_ < 0 || marker.id == marker_id_)) {
        target_found = true;
        target_pose.header.frame_id =
          camera_frame_.empty() ? msg->header.frame_id : camera_frame_;
        target_pose.header.stamp = msg->header.stamp;
        tf2::toMsg(transform, target_pose.pose);
      }
    }

    drawInfoPanel(image, info_lines);

    if (image_pub_.getNumSubscribers() > 0) {
      cv_bridge::CvImage out_msg;
      out_msg.header = msg->header;
      out_msg.encoding = sensor_msgs::image_encodings::RGB8;
      out_msg.image = image;
      image_pub_.publish(out_msg.toImageMsg());
    }

    if (target_found) {
      pose_pub_->publish(target_pose);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ArUco ID %d: x=%.3f y=%.3f z=%.3f m", marker_id_,
        target_pose.pose.position.x, target_pose.pose.position.y,
        target_pose.pose.position.z);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "检测到 %zu 个 ArUco 标记，但未找到目标 ID %d", markers_.size(), marker_id_);
    }
  }

  // 在图像左上角绘制半透明信息面板
  static void drawInfoPanel(cv::Mat & image, const std::vector<std::string> & lines)
  {
    if (lines.empty() || image.empty()) {
      return;
    }
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double scale = 0.5;
    const int thickness = 1;
    const int line_height = 22;
    const int pad = 8;

    int max_width = 0;
    for (const auto & line : lines) {
      int baseline = 0;
      cv::Size size = cv::getTextSize(line, font, scale, thickness, &baseline);
      max_width = std::max(max_width, size.width);
    }

    int box_w = std::min(max_width + 2 * pad, image.cols - 2 * pad);
    int box_h = std::min(static_cast<int>(lines.size()) * line_height + pad, image.rows - 2 * pad);
    cv::Rect box(pad, pad, box_w, box_h);

    cv::Mat overlay = image.clone();
    cv::rectangle(overlay, box, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::addWeighted(overlay, 0.55, image, 0.45, 0.0, image);

    for (std::size_t i = 0; i < lines.size(); ++i) {
      int y = pad + static_cast<int>(i + 1) * line_height - 5;
      if (y >= image.rows) {
        break;
      }
      cv::putText(
        image, lines[i], cv::Point(pad * 2, y), font, scale,
        cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
    }
  }

  aruco::CameraParameters camParam_;
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  bool cam_info_received_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string camera_frame_;
  double marker_size_;
  int marker_id_;
  bool image_is_rectified_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoPoseTest>();
  node->setup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
