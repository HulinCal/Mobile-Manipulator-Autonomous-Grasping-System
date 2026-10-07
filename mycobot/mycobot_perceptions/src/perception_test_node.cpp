/**
 * @file perception_test_node.cpp
 * @brief Test subscriber for the camera_perception node.
 *
 * This node subscribes to the perception output topics and reports statistics
 * (frame id, size, number of detected objects, target id, etc.) to the console.
 * It is used together with the perception_test.launch.py which also opens RViz
 * configured to display the published point clouds and markers.
 *
 * Run it with:
 *   ros2 run mycobot_perceptions perception_test_node
 * or via the launch file:
 *   ros2 launch mycobot_perceptions perception_test.launch.py
 *
 * @author hl
 * @date August 2026
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <std_msgs/msg/string.hpp>

namespace mycobot_perceptions {

class PerceptionTestNode : public rclcpp::Node {
 public:
  PerceptionTestNode() : rclcpp::Node("perception_test_node") {
    // Allow remapping the perception node namespace if needed.
    const std::string prefix = "/camera_perception";

    processed_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        prefix + "/processed_cloud", rclcpp::SensorDataQoS(),
        std::bind(&PerceptionTestNode::processedCloudCallback, this, std::placeholders::_1));

    objects_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        prefix + "/objects_cloud", rclcpp::SensorDataQoS(),
        std::bind(&PerceptionTestNode::objectsCloudCallback, this, std::placeholders::_1));

    support_plane_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        prefix + "/support_plane_cloud", rclcpp::SensorDataQoS(),
        std::bind(&PerceptionTestNode::supportPlaneCloudCallback, this, std::placeholders::_1));

    planning_scene_world_sub_ = this->create_subscription<moveit_msgs::msg::PlanningSceneWorld>(
        prefix + "/planning_scene_world",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&PerceptionTestNode::planningSceneCallback, this, std::placeholders::_1));

    // Subscribe to the per-object CollisionObject stream that the perception
    // node emits for direct consumption by MoveIt (operation=ADD on each msg).
    collision_object_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
        prefix + "/collision_object",
        rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable(),
        std::bind(&PerceptionTestNode::collisionObjectCallback, this, std::placeholders::_1));

    // Also listen on /planning_scene to confirm that move_group's planning
    // scene monitor is being fed the perceived scene diff.
    planning_scene_sub_ = this->create_subscription<moveit_msgs::msg::PlanningScene>(
        "/planning_scene",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&PerceptionTestNode::planningSceneDiffCallback, this, std::placeholders::_1));

    target_id_sub_ = this->create_subscription<std_msgs::msg::String>(
        prefix + "/target_object_id",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&PerceptionTestNode::targetIdCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "perception_test_node ready. Listening under '%s/' and '/planning_scene'. "
                "Open RViz to view the clouds.",
                prefix.c_str());
  }

 private:
  void processedCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "[processed_cloud] frame='%s' size=%ux%u (%zu bytes)",
                msg->header.frame_id.c_str(), msg->width, msg->height, msg->data.size());
  }

  void objectsCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "[objects_cloud]   frame='%s' points=%u (this is the cloud shown in RViz)",
                msg->header.frame_id.c_str(), msg->width * msg->height);
  }

  void supportPlaneCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "[support_plane]   frame='%s' points=%u",
                msg->header.frame_id.c_str(), msg->width * msg->height);
  }

  void planningSceneCallback(const moveit_msgs::msg::PlanningSceneWorld::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "[planning_scene_world]  collision_objects=%zu",
                msg->collision_objects.size());
    for (const auto& obj : msg->collision_objects) {
      RCLCPP_INFO(this->get_logger(), "  - id='%s' frame='%s' primitives=%zu operation=%d",
                  obj.id.c_str(), obj.header.frame_id.c_str(),
                  obj.primitives.size(), static_cast<int>(obj.operation));
    }
  }

  void collisionObjectCallback(const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
    // Each message here is a complete, MoveIt-ready CollisionObject: id +
    // primitive(s) + pose + operation=ADD. Republishing this on
    // /collision_object (or feeding it to applyPlanningScene) is sufficient
    // to add the object to the planning scene.
    RCLCPP_INFO(this->get_logger(),
                "[collision_object] id='%s' frame='%s' primitives=%zu operation=%d",
                msg->id.c_str(), msg->header.frame_id.c_str(),
                msg->primitives.size(), static_cast<int>(msg->operation));
  }

  void planningSceneDiffCallback(const moveit_msgs::msg::PlanningScene::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),
                "[/planning_scene] is_diff=%d collision_objects=%zu (consumed by move_group PSM)",
                static_cast<int>(msg->is_diff), msg->world.collision_objects.size());
  }

  void targetIdCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "[target_object_id] '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr processed_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr objects_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr support_plane_cloud_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningSceneWorld>::SharedPtr planning_scene_world_sub_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_sub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_id_sub_;
};

}  // namespace mycobot_perceptions

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mycobot_perceptions::PerceptionTestNode>());
  rclcpp::shutdown();
  return 0;
}
