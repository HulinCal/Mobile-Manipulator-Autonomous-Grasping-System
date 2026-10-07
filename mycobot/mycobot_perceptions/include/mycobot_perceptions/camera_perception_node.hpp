/**
 * @file camera_perception_node.hpp
 * @brief Streaming perception node for the myCobot pick-and-place project.
 *
 * CameraPerceptionNode subscribes to an RGB-D camera (organised point cloud +
 * colour image), runs the same segmentation pipeline used by
 * GetPlanningSceneServer in mycobot_mtc_pick_place_demo, and publishes the
 * results as topics. Unlike the reference (which is request/response via a
 * service), this node is a *streaming* perception source: it processes the
 * latest camera data on a configurable timer and advertises:
 *
 *   ~/processed_cloud        (sensor_msgs/PointCloud2)          - transformed + cropped cloud
 *   ~/objects_cloud          (sensor_msgs/PointCloud2)          - points above the support plane
 *   ~/support_plane_cloud    (sensor_msgs/PointCloud2)          - support plane points
 *   ~/planning_scene_world   (moveit_msgs/PlanningSceneWorld)  - detected CollisionObjects (latched)
 *   ~/target_object_id       (std_msgs/String)                  - id of the best target match (latched)
 *   ~/support_surface_id     (std_msgs/String)                  - id of the support surface (latched)
 *   ~/object_markers         (visualization_msgs/MarkerArray)  - RViz markers
 *
 * The target shape and dimensions are parameters (not a service request) so
 * downstream grasp planners can either use the published target id directly or
 * re-run identification with their own dimensions.
 *
 * @author hl
 * @date August 2026
 */

#ifndef MYCOBOT_PERCEPTIONS_CAMERA_PERCEPTION_NODE_HPP
#define MYCOBOT_PERCEPTIONS_CAMERA_PERCEPTION_NODE_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mycobot_perceptions/perception_utils.hpp"

namespace mycobot_perceptions {

class CameraPerceptionNode : public rclcpp::Node {
 public:
  explicit CameraPerceptionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // Configuration
  PerceptionParams params_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_image_sub_;

  // Publishers
  // --- point cloud outputs (for visualization in rviz / debugging) ---
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr objects_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr support_plane_cloud_pub_;

  // --- MoveIt scene outputs (directly consumable by the planning scene) ---
  // ~/planning_scene_world: full PlanningSceneWorld (latched). Suitable for
  //     one-shot snapshot reads by grasp planners.
  // ~/collision_object: stream of individual CollisionObject messages with
  //     operation=ADD. Each message is a complete, self-contained description
  //     of one object and can be fed straight into moveit's
  //     /collision_object input or applyPlanningScene service.
  // ~/planning_scene: full PlanningScene message (latched). Published on the
  //     latched QoS so move_group's planning scene monitor, which subscribes
  //     to /planning_scene, automatically applies the scene update when this
  //     node's ~/planning_scene is bridged onto /planning_scene via remapping.
  rclcpp::Publisher<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr planning_scene_world_pub_;
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_pub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

  // --- Identification outputs ---
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_object_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr support_surface_id_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr object_markers_pub_;

  // Processing timer
  rclcpp::TimerBase::SharedPtr processing_timer_;

  // Latest sensor data
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud_;
  sensor_msgs::msg::Image::SharedPtr latest_rgb_image_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Setup helpers ---
  void declareAndLoadParameters();
  void createSubscribers();
  void createPublishers();
  void createProcessingTimer();

  // --- Callbacks ---
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // --- Core pipeline ---
  void processTimerCallback();
  void runPerceptionPipeline();
};

}  // namespace mycobot_perceptions

#endif  // MYCOBOT_PERCEPTIONS_CAMERA_PERCEPTION_NODE_HPP
