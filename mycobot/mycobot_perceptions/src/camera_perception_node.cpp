/**
 * @file camera_perception_node.cpp
 * @brief Implementation of the streaming camera perception node.
 *
 * @author hl
 * @date August 2026
 */

#include "mycobot_perceptions/camera_perception_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

#include "mycobot_mtc_pick_place_demo/cluster_extraction.h"
#include "mycobot_mtc_pick_place_demo/normals_curvature_and_rsd_estimation.h"
#include "mycobot_mtc_pick_place_demo/object_segmentation.h"
#include "mycobot_mtc_pick_place_demo/plane_segmentation.h"

namespace mycobot_perceptions {

CameraPerceptionNode::CameraPerceptionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("camera_perception", options) {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declareAndLoadParameters();
  createSubscribers();
  createPublishers();
  createProcessingTimer();

  RCLCPP_INFO(this->get_logger(),
              "camera_perception node ready. Subscribed to '%s' and '%s'.",
              params_.point_cloud_topic.c_str(), params_.rgb_image_topic.c_str());
}

void CameraPerceptionNode::declareAndLoadParameters() {
  auto declare_param = [this](const std::string& name, const auto& default_value,
                              const std::string& description = "") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_value, descriptor);
    }
  };

  // ---- Input topics & frame ----
  declare_param("point_cloud_topic", std::string("/camera/depth_registered/points"),
                "Topic name for incoming point cloud data");
  declare_param("rgb_image_topic", std::string("/camera/color/image_raw"),
                "Topic name for incoming RGB image data");
  declare_param("target_frame", std::string("base_link"), "Target frame for processing");

  // ---- Processing ----
  declare_param("processing_rate", 0.5, "How often (Hz) to run the perception pipeline");
  declare_param("publish_processed_cloud", true, "Publish the transformed/cropped cloud");
  declare_param("publish_objects_cloud", true, "Publish the objects cloud");
  declare_param("publish_support_plane_cloud", true, "Publish the support plane cloud");
  declare_param("publish_object_markers", true, "Publish RViz markers for detected objects");

  // ---- Crop box ----
  declare_param("enable_cropping", true, "Enable point cloud cropping");
  declare_param("crop_min_x", 0.10, "Minimum X value for crop box");
  declare_param("crop_max_x", 1.10, "Maximum X value for crop box");
  declare_param("crop_min_y", 0.00, "Minimum Y value for crop box");
  declare_param("crop_max_y", 0.90, "Maximum Y value for crop box");
  declare_param("crop_min_z", -std::numeric_limits<double>::infinity(), "Minimum Z value for crop box");
  declare_param("crop_max_z", std::numeric_limits<double>::infinity(), "Maximum Z value for crop box");

  // ---- Plane & object segmentation ----
  declare_param("max_iterations", 100, "Maximum iterations for RANSAC");
  declare_param("distance_threshold", 0.01, "Distance threshold for RANSAC");
  declare_param("z_tolerance", 0.03, "Tolerance for z-coordinate of the support plane");
  declare_param("angle_tolerance", 0.9990482216, "Angle tolerance for surface normals");
  declare_param("plane_segmentation_threshold", 0.001, "Threshold for plane segmentation");
  declare_param("min_cluster_size", 150, "Minimum size of a cluster");
  declare_param("max_cluster_size", 1000000, "Maximum size of a cluster");
  declare_param("cluster_tolerance", 0.02, "Tolerance for Euclidean clustering");
  declare_param("normal_estimation_k", 30, "Number of neighbors for normal estimation");
  declare_param("w_inliers", 1.0, "Weight for inlier score");
  declare_param("w_size", 1.0, "Weight for size score");
  declare_param("w_distance", 1.0, "Weight for distance score");
  declare_param("w_orientation", 1.0, "Weight for orientation score");
  declare_param("max_plane_segmentation_iterations", 1000, "Max iterations for plane segmentation RANSAC");
  declare_param("plane_segmentation_distance_threshold", 0.01, "Distance threshold for plane segmentation (m)");

  // ---- Support surface ----
  declare_param("support_surface_name", std::string("support_surface"), "Name of the support surface collision object");
  declare_param("min_surface_thickness", 0.0001, "Minimum thickness for the support surface (m)");

  // ---- Normal / curvature / RSD ----
  declare_param("k_neighbors", 30, "Number of neighbors for normal estimation");
  declare_param("max_plane_error", 0.01, "Threshold for MLESAC plane fitting");
  declare_param("max_iterations_normals", 100, "Max iterations for MLESAC in normal estimation");
  declare_param("min_boundary_neighbors", 10, "Minimum neighbors for boundary points");
  declare_param("rsd_radius", 0.02, "Radius for RSD estimation");

  // ---- Cluster extraction ----
  declare_param("nearest_neighbors", 30, "Number of neighbors for region growing");
  declare_param("smoothness_threshold", 20.0, "Smoothness threshold (degrees)");
  declare_param("curvature_threshold", 0.2, "Curvature threshold");

  // ---- Object segmentation ----
  declare_param("num_iterations", 5, "Number of iterations for the inner loop");
  declare_param("inlier_threshold", 85, "Inlier count to consider a model valid");
  declare_param("hough_radius_bins", 50, "Number of radius bins for circle Hough space");
  declare_param("hough_center_bins", 50, "Number of center bins for circle Hough space");
  declare_param("ransac_distance_threshold", 0.001, "Distance threshold for RANSAC");
  declare_param("ransac_max_iterations", 1000, "Max iterations for RANSAC");
  declare_param("circle_min_cluster_size", 20, "Min size for a cluster of circle inliers");
  declare_param("circle_max_clusters", 2, "Max number of allowed clusters for circles");
  declare_param("circle_height_tolerance", 0.025, "Tolerance for height difference between circle clusters");
  declare_param("circle_curvature_threshold", 0.0015, "Threshold for point curvature in circle fitting");
  declare_param("circle_radius_tolerance", 0.020, "Tolerance for RSD min value vs circle radius");
  declare_param("circle_normal_angle_threshold", 0.2, "Threshold for normal vs circle radial vector angle");
  declare_param("circle_cluster_tolerance", 0.025, "Max distance between two points in the same circle cluster");
  declare_param("line_min_cluster_size", 20, "Min points for a valid line cluster");
  declare_param("line_max_clusters", 1, "Max number of allowed clusters for lines");
  declare_param("line_curvature_threshold", 0.0011, "Threshold for point curvature in line fitting");
  declare_param("line_cluster_tolerance", 0.025, "Max distance between two points in the same line cluster");
  declare_param("line_rho_threshold", 0.01, "Tolerance for rho when clustering line models");
  declare_param("line_theta_threshold", 0.1, "Tolerance for theta when clustering line models");

  // ---- Legacy shape fitting ----
  declare_param("shape_fitting_max_iterations", 1000, "Max iterations for shape fitting RANSAC");
  declare_param("shape_fitting_distance_threshold", 0.01, "Distance threshold for shape fitting (m)");
  declare_param("shape_fitting_min_radius", 0.01, "Min radius for cylinder fitting (m)");
  declare_param("shape_fitting_max_radius", 0.1, "Max radius for cylinder fitting (m)");
  declare_param("shape_fitting_normal_distance_weight", 0.1, "Normal distance weight for cylinder fitting");
  declare_param("shape_fitting_normal_search_radius", 0.05, "Search radius for normal estimation in shape fitting (m)");

  // ---- Target object description ----
  declare_param("target_shape", std::string("cylinder"),
                "Target object shape ('cylinder' or 'box')");
  declare_param("target_dimensions", std::vector<double>{0.06, 0.0125},
                "Approximate target object dimensions (height, radius) or (x, y, z)");

  // ---- Debug ----
  // (PCD I/O intentionally disabled; publish clouds as ROS topics instead.)
  declare_param("output_directory", std::string("/tmp/"), "Reserved (no longer used; PCD I/O disabled)");
  declare_param("debug_pcd_filename", std::string("debug_cloud.pcd"), "Reserved (no longer used)");
  declare_param("save_debug_pcd", false, "Reserved (no longer used; PCD I/O disabled)");

  // ---- Load values into the params struct ----
  auto get = [this](const std::string& name) { return this->get_parameter(name).get_value<rclcpp::Parameter>(); };

  params_.point_cloud_topic = get("point_cloud_topic").as_string();
  params_.rgb_image_topic = get("rgb_image_topic").as_string();
  params_.target_frame = get("target_frame").as_string();

  params_.processing_rate = get("processing_rate").as_double();
  params_.publish_processed_cloud = get("publish_processed_cloud").as_bool();
  params_.publish_objects_cloud = get("publish_objects_cloud").as_bool();
  params_.publish_support_plane_cloud = get("publish_support_plane_cloud").as_bool();
  params_.publish_object_markers = get("publish_object_markers").as_bool();

  params_.enable_cropping = get("enable_cropping").as_bool();
  params_.crop_min_x = get("crop_min_x").as_double();
  params_.crop_max_x = get("crop_max_x").as_double();
  params_.crop_min_y = get("crop_min_y").as_double();
  params_.crop_max_y = get("crop_max_y").as_double();
  params_.crop_min_z = get("crop_min_z").as_double();
  params_.crop_max_z = get("crop_max_z").as_double();

  params_.max_iterations = get("max_iterations").as_int();
  params_.distance_threshold = get("distance_threshold").as_double();
  params_.z_tolerance = get("z_tolerance").as_double();
  params_.angle_tolerance = get("angle_tolerance").as_double();
  params_.plane_segmentation_threshold = get("plane_segmentation_threshold").as_double();
  params_.min_cluster_size = get("min_cluster_size").as_int();
  params_.max_cluster_size = get("max_cluster_size").as_int();
  params_.cluster_tolerance = get("cluster_tolerance").as_double();
  params_.normal_estimation_k = get("normal_estimation_k").as_int();
  params_.w_inliers = get("w_inliers").as_double();
  params_.w_size = get("w_size").as_double();
  params_.w_distance = get("w_distance").as_double();
  params_.w_orientation = get("w_orientation").as_double();
  params_.max_plane_segmentation_iterations = get("max_plane_segmentation_iterations").as_int();
  params_.plane_segmentation_distance_threshold = get("plane_segmentation_distance_threshold").as_double();

  params_.support_surface_name = get("support_surface_name").as_string();
  params_.min_surface_thickness = get("min_surface_thickness").as_double();

  params_.k_neighbors = get("k_neighbors").as_int();
  params_.max_plane_error = get("max_plane_error").as_double();
  params_.max_iterations_normals = get("max_iterations_normals").as_int();
  params_.min_boundary_neighbors = get("min_boundary_neighbors").as_int();
  params_.rsd_radius = get("rsd_radius").as_double();

  params_.nearest_neighbors = get("nearest_neighbors").as_int();
  params_.smoothness_threshold = static_cast<float>(get("smoothness_threshold").as_double());
  params_.curvature_threshold = static_cast<float>(get("curvature_threshold").as_double());

  params_.num_iterations = get("num_iterations").as_int();
  params_.inlier_threshold = get("inlier_threshold").as_int();
  params_.hough_radius_bins = get("hough_radius_bins").as_int();
  params_.hough_center_bins = get("hough_center_bins").as_int();
  params_.ransac_distance_threshold = get("ransac_distance_threshold").as_double();
  params_.ransac_max_iterations = get("ransac_max_iterations").as_int();
  params_.circle_min_cluster_size = get("circle_min_cluster_size").as_int();
  params_.circle_max_clusters = get("circle_max_clusters").as_int();
  params_.circle_height_tolerance = get("circle_height_tolerance").as_double();
  params_.circle_curvature_threshold = get("circle_curvature_threshold").as_double();
  params_.circle_radius_tolerance = get("circle_radius_tolerance").as_double();
  params_.circle_normal_angle_threshold = get("circle_normal_angle_threshold").as_double();
  params_.circle_cluster_tolerance = get("circle_cluster_tolerance").as_double();
  params_.line_min_cluster_size = get("line_min_cluster_size").as_int();
  params_.line_max_clusters = get("line_max_clusters").as_int();
  params_.line_curvature_threshold = get("line_curvature_threshold").as_double();
  params_.line_cluster_tolerance = get("line_cluster_tolerance").as_double();
  params_.line_rho_threshold = get("line_rho_threshold").as_double();
  params_.line_theta_threshold = get("line_theta_threshold").as_double();

  params_.shape_fitting_max_iterations = get("shape_fitting_max_iterations").as_int();
  params_.shape_fitting_distance_threshold = get("shape_fitting_distance_threshold").as_double();
  params_.shape_fitting_min_radius = get("shape_fitting_min_radius").as_double();
  params_.shape_fitting_max_radius = get("shape_fitting_max_radius").as_double();
  params_.shape_fitting_normal_distance_weight = get("shape_fitting_normal_distance_weight").as_double();
  params_.shape_fitting_normal_search_radius = get("shape_fitting_normal_search_radius").as_double();

  params_.target_shape = get("target_shape").as_string();
  params_.target_dimensions = get("target_dimensions").as_double_array();

  params_.output_directory = get("output_directory").as_string();
  params_.debug_pcd_filename = get("debug_pcd_filename").as_string();
  params_.save_debug_pcd = get("save_debug_pcd").as_bool();

   params_.target_frame = "camera_link";
}

void CameraPerceptionNode::createSubscribers() {
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.point_cloud_topic, 10,
      std::bind(&CameraPerceptionNode::pointCloudCallback, this, std::placeholders::_1));
  rgb_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      params_.rgb_image_topic, 10,
      std::bind(&CameraPerceptionNode::rgbImageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to point cloud topic: %s", params_.point_cloud_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribed to RGB image topic: %s", params_.rgb_image_topic.c_str());
}

void CameraPerceptionNode::createPublishers() {
  processed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/processed_cloud", rclcpp::SensorDataQoS());
  objects_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/objects_cloud", rclcpp::SensorDataQoS());
  support_plane_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/support_plane_cloud", rclcpp::SensorDataQoS());

  // Latched (transient_local) so a late subscriber (e.g. grasp planner launched
  // later) immediately receives the most recent perceived scene.
  planning_scene_world_pub_ = this->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
      "~/planning_scene_world", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Stream of individual CollisionObjects. operation is set to ADD on each
  // message so a downstream node that simply republishes onto /collision_object
  // (or moveit's internal API consumer) gets ready-to-apply objects.
  collision_object_pub_ = this->create_publisher<moveit_msgs::msg::CollisionObject>(
      "~/collision_object", rclcpp::QoS(rclcpp::KeepLast(50)).transient_local().reliable());

  // Full PlanningScene. Moveit's PlanningSceneMonitor subscribes to
  // /planning_scene and applies any received scene as a diff update. Remap
  // ~/planning_scene -> /planning_scene when you want move_group to consume
  // the perceived scene directly (see perception_test.launch.py).
  planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
      "~/planning_scene", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  target_object_id_pub_ = this->create_publisher<std_msgs::msg::String>(
      "~/target_object_id", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  support_surface_id_pub_ = this->create_publisher<std_msgs::msg::String>(
      "~/support_surface_id", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  object_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/object_markers", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void CameraPerceptionNode::createProcessingTimer() {
  if (params_.processing_rate <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "processing_rate <= 0; pipeline disabled (manual trigger only).");
    return;
  }
  const auto period = std::chrono::duration<double>(1.0 / params_.processing_rate);
  processing_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CameraPerceptionNode::processTimerCallback, this));
  RCLCPP_INFO(this->get_logger(), "Processing pipeline running at %.3f Hz", params_.processing_rate);
}

void CameraPerceptionNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (msg && !msg->data.empty()) {
    latest_point_cloud_ = msg;
  }
}

void CameraPerceptionNode::rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (msg && !msg->data.empty()) {
    latest_rgb_image_ = msg;
  }
}

void CameraPerceptionNode::processTimerCallback() {
  // Guard against re-entrancy: the pipeline can take longer than the period.
  static std::atomic<bool> processing{false};
  bool expected = false;
  if (!processing.compare_exchange_strong(expected, true)) {
    return;
  }
  runPerceptionPipeline();
  processing.store(false);
}

void CameraPerceptionNode::runPerceptionPipeline() {
  const auto logger = this->get_logger();
  const rclcpp::Time stamp = this->now();

  // 1. Check data availability
  if (!latest_point_cloud_ || !latest_rgb_image_) {
    RCLCPP_WARN_THROTTLE(logger, *this->get_clock(), 5000,
                        "Point cloud or RGB image not yet available; skipping pipeline.");
    return;
  }

  // 2. Transform + crop
  auto transformed_cloud = transformPointCloud(latest_point_cloud_, params_.target_frame,
                                               *tf_buffer_, params_, logger, stamp);
  if (!transformed_cloud) {
    RCLCPP_ERROR(logger, "Failed to transform point cloud to target frame");
    return;
  }

  if (params_.publish_processed_cloud) {
    processed_cloud_pub_->publish(*transformed_cloud);
  }

  // 3. Convert to PCL
  auto pcl_cloud = convertToPCL(transformed_cloud, logger);
  if (!pcl_cloud) {
    RCLCPP_ERROR(logger, "Failed to convert PointCloud2 to PCL format");
    return;
  }

  // 4. Segment support plane and objects
  auto [support_plane_cloud, objects_cloud, plane_coefficients] = segmentPlaneAndObjects(
      pcl_cloud, params_.enable_cropping, params_.crop_min_x, params_.crop_max_x,
      params_.crop_min_y, params_.crop_max_y, params_.crop_min_z, params_.crop_max_z,
      params_.max_plane_segmentation_iterations, params_.plane_segmentation_distance_threshold,
      params_.z_tolerance, params_.angle_tolerance, params_.min_cluster_size,
      params_.max_cluster_size, params_.cluster_tolerance, params_.normal_estimation_k,
      params_.plane_segmentation_threshold, params_.w_inliers, params_.w_size,
      params_.w_distance, params_.w_orientation);

  if (!support_plane_cloud || !objects_cloud || !plane_coefficients) {
    RCLCPP_ERROR(logger, "Plane and object segmentation failed");
    return;
  }
  RCLCPP_INFO(logger, "Segmentation OK: plane=%zu pts, objects=%zu pts",
              support_plane_cloud->size(), objects_cloud->size());

  // Publish the point clouds so the test node / rviz can visualise them.
  if (params_.publish_support_plane_cloud && !support_plane_cloud->empty()) {
    sensor_msgs::msg::PointCloud2 plane_msg;
    pcl::toROSMsg(*support_plane_cloud, plane_msg);
    plane_msg.header.frame_id = params_.target_frame;
    plane_msg.header.stamp = stamp;
    support_plane_cloud_pub_->publish(plane_msg);
  }
  if (params_.publish_objects_cloud && !objects_cloud->empty()) {
    sensor_msgs::msg::PointCloud2 objects_msg;
    pcl::toROSMsg(*objects_cloud, objects_msg);
    objects_msg.header.frame_id = params_.target_frame;
    objects_msg.header.stamp = stamp;
    objects_cloud_pub_->publish(objects_msg);
  }

  // 5. Build support surface CollisionObject
  moveit_msgs::msg::CollisionObject support_surface = createSupportSurfaceObject(
      support_plane_cloud, plane_coefficients, params_.target_frame, params_, logger, stamp);

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::string support_surface_id;
  if (!support_surface.id.empty()) {
    collision_objects.push_back(support_surface);
    support_surface_id = support_surface.id;
  }

  // 6. Estimate normals / curvature / RSD
  auto cloud_with_features = estimateNormalsCurvatureAndRSD(
      objects_cloud, params_.k_neighbors, params_.max_plane_error,
      params_.max_iterations_normals, params_.min_boundary_neighbors, params_.rsd_radius);
  if (!cloud_with_features || cloud_with_features->empty()) {
    RCLCPP_ERROR(logger, "Failed to estimate normals, curvature, and RSD");
    return;
  }
  RCLCPP_INFO(logger, "Estimated normals/curvature/RSD for %zu points", cloud_with_features->size());

  // 7. Extract clusters
  std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr> clusters = extractClusters(
      cloud_with_features, static_cast<unsigned int>(params_.min_cluster_size),
      static_cast<unsigned int>(params_.max_cluster_size), params_.smoothness_threshold,
      params_.curvature_threshold, static_cast<unsigned int>(params_.nearest_neighbors));
  if (clusters.empty()) {
    RCLCPP_ERROR(logger, "Failed to extract any clusters from the point cloud");
    return;
  }
  RCLCPP_INFO(logger, "Extracted %zu clusters", clusters.size());

  // 8. Segment objects (cylinder/box fitting via Hough + RANSAC)
  std::vector<moveit_msgs::msg::CollisionObject> segmented_objects = segmentObjects(
      clusters, params_.num_iterations, params_.target_frame, params_.inlier_threshold,
      params_.hough_radius_bins, params_.hough_center_bins, params_.ransac_distance_threshold,
      params_.ransac_max_iterations, params_.circle_min_cluster_size, params_.circle_max_clusters,
      params_.circle_height_tolerance, params_.circle_curvature_threshold,
      params_.circle_radius_tolerance, params_.circle_normal_angle_threshold,
      params_.circle_cluster_tolerance, params_.line_min_cluster_size, params_.line_max_clusters,
      params_.line_curvature_threshold, params_.line_cluster_tolerance, params_.line_rho_threshold,
      params_.line_theta_threshold);
  RCLCPP_INFO(logger, "Segmented %zu objects", segmented_objects.size());
  for (const auto& obj : segmented_objects) {
    collision_objects.push_back(obj);
  }

  // 9. Identify target object
  const std::string target_object_id = identifyTargetObject(
      collision_objects, params_.target_shape, params_.target_dimensions, logger);

  // 10. Assemble + publish PlanningSceneWorld
  moveit_msgs::msg::PlanningSceneWorld scene_world;
  for (const auto& obj : collision_objects) {
    scene_world.collision_objects.push_back(obj);
  }
  planning_scene_world_pub_->publish(scene_world);

  // 11. Publish each CollisionObject individually as well. Each message is
  // self-contained (id, primitives, primitive_poses, operation=ADD) and can
  // be republished verbatim onto moveit's /collision_object topic, or pushed
  // through the applyPlanningScene service.
  for (const auto& obj : collision_objects) {
    collision_object_pub_->publish(obj);
  }

  // 12. Publish a full PlanningScene (as a diff). When ~/planning_scene is
  // remapped onto /planning_scene, move_group's PlanningSceneMonitor will
  // apply this scene to its internal planning scene automatically, making
  // the perceived objects visible to motion planning as collision objects
  // and to grasp planners as the target set.
  moveit_msgs::msg::PlanningScene planning_scene_msg;
  planning_scene_msg.is_diff = true;
  planning_scene_msg.world = scene_world;
  planning_scene_msg.robot_state.is_diff = true;
  planning_scene_pub_->publish(planning_scene_msg);

  // Publish IDs (latched so late subscribers get the latest result)
  std_msgs::msg::String id_msg;
  id_msg.data = target_object_id;
  target_object_id_pub_->publish(id_msg);
  id_msg.data = support_surface_id;
  support_surface_id_pub_->publish(id_msg);

  // 13. RViz markers
  if (params_.publish_object_markers) {
    auto markers = createObjectMarkers(collision_objects, target_object_id, stamp);
    object_markers_pub_->publish(markers);
  }

  RCLCPP_INFO(logger,
              "Pipeline complete: %zu CollisionObjects published (target='%s', support='%s'); "
              "PlanningScene diff published.",
              collision_objects.size(), target_object_id.c_str(),
              support_surface_id.c_str());
}

}  // namespace mycobot_perceptions

int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<mycobot_perceptions::CameraPerceptionNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_perception"), "Caught exception: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_perception"), "Caught unknown exception");
    return 1;
  }
  return 0;
}
