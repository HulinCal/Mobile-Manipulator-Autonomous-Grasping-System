/**
 * @file grasp_node.cpp
 * @brief 订阅 ArUco MarkerArray，计算 MoveIt 目标位姿并打印。
 *
 * 公式（详见 gripper_object_pose.md）：
 *   T_base_marker = T_base_cam × T_cam_marker
 *   T_base_gripper = T_base_marker × T_grasp⁻¹
 *
 * 其中 T_grasp = gripper_base → marker，是从上往下接近目标的常量变换。
 *
 * @author hl
 * @date September 2026
 */

#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "aruco_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class GraspNode : public rclcpp::Node
{
public:
  GraspNode() : Node("grasp_node")
  {
    // ---- 参数声明 ----
    this->declare_parameter<std::vector<int64_t>>("target_marker_ids", {1, 2, 3});
    this->declare_parameter<std::vector<double>>("grasp_offset", {0.0, 0.10, 0.0});
    this->declare_parameter<std::vector<double>>("grasp_orientation", {0.5, -0.5, 0.5, 0.5});
    this->declare_parameter<std::string>("robot_base_frame", "base_link");
    this->declare_parameter<std::string>("camera_optical_frame", "camera_color_optical_frame");

    // ---- 读取参数 ----
    target_ids_ = this->get_parameter("target_marker_ids").as_integer_array();
    auto offset = this->get_parameter("grasp_offset").as_double_array();
    auto quat = this->get_parameter("grasp_orientation").as_double_array();
    base_frame_ = this->get_parameter("robot_base_frame").as_string();
    camera_frame_ = this->get_parameter("camera_optical_frame").as_string();

    // ---- 构建 T_grasp (gripper_base → marker) ----
    if (offset.size() < 3 || quat.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "grasp_offset 需要 3 个值，grasp_orientation 需要 4 个值");
      return;
    }
    T_grasp_.setOrigin(tf2::Vector3(offset[0], offset[1], offset[2]));
    T_grasp_.setRotation(tf2::Quaternion(quat[0], quat[1], quat[2], quat[3]));

    RCLCPP_INFO(this->get_logger(), "==== GraspNode 已启动 ====");
    RCLCPP_INFO(this->get_logger(), "目标 marker IDs: [%s]",
      formatIds(target_ids_).c_str());
    RCLCPP_INFO(this->get_logger(), "T_grasp 平移: (%.3f, %.3f, %.3f)",
      offset[0], offset[1], offset[2]);
    RCLCPP_INFO(this->get_logger(), "T_grasp 四元数: (%.3f, %.3f, %.3f, %.3f)",
      quat[0], quat[1], quat[2], quat[3]);
    RCLCPP_INFO(this->get_logger(), "base_frame='%s', camera_frame='%s'",
      base_frame_.c_str(), camera_frame_.c_str());

    // ---- TF ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- 订阅 MarkerArray ----
    // marker_publisher 的 sub-node 默认命名为 /aruco_marker_publisher
    markers_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
        "/aruco_marker_publisher/markers", 10,
        std::bind(&GraspNode::markersCallback, this, std::placeholders::_1));

    // ---- 发布 ----
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/grasp_target_pose", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/grasp_target_viz", 10);
  }

private:
  void markersCallback(const aruco_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // 1. 查询 T_base_cam (base_link → camera_color_optical_frame)
    geometry_msgs::msg::TransformStamped tf_base_cam;
    try {
      tf_base_cam = tf_buffer_->lookupTransform(
          base_frame_, camera_frame_,
          tf2::TimePointZero, std::chrono::milliseconds(100));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "无法获取 %s → %s 的 TF: %s",
          base_frame_.c_str(), camera_frame_.c_str(), e.what());
      return;
    }

    tf2::Transform T_base_cam;
    tf2::fromMsg(tf_base_cam.transform, T_base_cam);

    tf2::Transform T_grasp_inv = T_grasp_.inverse();

    // 2. 遍历检测到的 markers
    visualization_msgs::msg::MarkerArray viz_msg;

    for (const auto & marker : msg->markers) {
      // 检查 ID 是否在目标列表中
      if (std::find(target_ids_.begin(), target_ids_.end(),
            static_cast<int64_t>(marker.id)) == target_ids_.end()) {
        continue;
      }

      // 3. T_cam_marker：marker 在相机光学帧下的位姿
      tf2::Transform T_cam_marker;
      tf2::fromMsg(marker.pose.pose, T_cam_marker);

      // 4. T_base_marker = T_base_cam × T_cam_marker
      tf2::Transform T_base_marker = T_base_cam * T_cam_marker;

      // 5. T_base_gripper = T_base_marker × T_grasp⁻¹
      tf2::Transform T_base_gripper = T_base_marker * T_grasp_inv;

      // 6. 转换为 PoseStamped 并打印
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = this->now();
      pose_msg.header.frame_id = base_frame_;
      pose_msg.pose.position.x = T_base_gripper.getOrigin().x();
      pose_msg.pose.position.y = T_base_gripper.getOrigin().y();
      pose_msg.pose.position.z = T_base_gripper.getOrigin().z();
      pose_msg.pose.orientation.x = T_base_gripper.getRotation().x();
      pose_msg.pose.orientation.y = T_base_gripper.getRotation().y();
      pose_msg.pose.orientation.z = T_base_gripper.getRotation().z();
      pose_msg.pose.orientation.w = T_base_gripper.getRotation().w();

      // 打印 MoveIt 目标位姿
      RCLCPP_INFO(this->get_logger(),
          "=== Marker ID %d → MoveIt 目标位姿 (gripper_base in %s) ===",
          marker.id, base_frame_.c_str());

      RCLCPP_INFO(this->get_logger(),
          "相机下的位姿， position:    x=%.4f, y=%.4f, z=%.4f [m]",
          T_cam_marker.getOrigin().x(), T_cam_marker.getOrigin().y(), T_cam_marker.getOrigin().z());

      RCLCPP_INFO(this->get_logger(),
          " 转换到base下的位姿， position:    x=%.4f, y=%.4f, z=%.4f [m]",
          pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
      // RCLCPP_INFO(this->get_logger(),
      //     "  orientation: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
      //     pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
      //     pose_msg.pose.orientation.z, pose_msg.pose.orientation.w);

      // 打印 RPY 以便直观理解
      tf2::Matrix3x3 m(T_base_gripper.getRotation());
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      // RCLCPP_INFO(this->get_logger(),
      //     "  RPY (rad):   r=%.4f, p=%.4f, y=%.4f", roll, pitch, yaw);
      // RCLCPP_INFO(this->get_logger(),
      //     "  RPY (deg):   r=%.1f, p=%.1f, y=%.1f",
      //     roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

      // 也打印 marker 在 base 下的位姿（参考用）
      RCLCPP_INFO(this->get_logger(),
          "  (marker in base: x=%.4f, y=%.4f, z=%.4f [m])",
          T_base_marker.getOrigin().x(),
          T_base_marker.getOrigin().y(),
          T_base_marker.getOrigin().z());

      pose_pub_->publish(pose_msg);

      // 可视化 marker（红色球）
      visualization_msgs::msg::Marker viz_marker;
      viz_marker.header = pose_msg.header;
      viz_marker.ns = "grasp_target";
      viz_marker.id = marker.id;
      viz_marker.type = visualization_msgs::msg::Marker::ARROW;
      viz_marker.action = visualization_msgs::msg::Marker::ADD;
      viz_marker.pose = pose_msg.pose;
      viz_marker.scale.x = 0.05;  // 箭头长度
      viz_marker.scale.y = 0.01;  // 宽度
      viz_marker.scale.z = 0.01;
      viz_marker.color.r = 0.0;
      viz_marker.color.g = 1.0;
      viz_marker.color.b = 0.0;
      viz_marker.color.a = 1.0;
      viz_marker.lifetime = rclcpp::Duration(1, 0);
      viz_msg.markers.push_back(viz_marker);
    }

    if (!viz_msg.markers.empty()) {
      viz_pub_->publish(viz_msg);
    }
  }

  std::string formatIds(const std::vector<int64_t> & ids) {
    std::string s;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) s += ", ";
      s += std::to_string(ids[i]);
    }
    return s;
  }

  // ---- 成员变量 ----
  std::vector<int64_t> target_ids_;
  std::string base_frame_;
  std::string camera_frame_;
  tf2::Transform T_grasp_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GraspNode>());
  rclcpp::shutdown();
  return 0;
}
