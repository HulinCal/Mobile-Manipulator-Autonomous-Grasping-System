/**
 * @file perception_utils.hpp
 * @brief Pure perception helpers for the mycobot_perceptions package.
 *
 * These functions implement the scene-understanding pipeline that the
 * camera_perception node uses. They are kept free of ROS node state so that
 * they can be reused by other nodes (e.g. a future real-arm grasping node)
 * or unit-tested in isolation.
 *
 * The heavy segmentation algorithms themselves live in the
 * mycobot_mtc_pick_place_demo package (plane_segmentation, cluster_extraction,
 * normals_curvature_and_rsd_estimation, object_segmentation) and are linked
 * against this library. This file only adds the glue: TF transformation,
 * cropping, CollisionObject assembly, target identification and RViz markers.
 *
 * @author hl
 * @date August 2026
 */

#ifndef MYCOBOT_PERCEPTIONS_PERCEPTION_UTILS_HPP
#define MYCOBOT_PERCEPTIONS_PERCEPTION_UTILS_HPP

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/buffer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "mycobot_mtc_pick_place_demo/normals_curvature_and_rsd_estimation.h"  // PointXYZRGBNormalRSD

namespace mycobot_perceptions {

/**
 * @brief Bundle of all perception parameters.
 *
 * Loaded once from the ROS parameter server and passed by const reference to
 * the utility functions. Keeping every knob in a single struct makes the
 * pipeline easy to configure and keeps function signatures readable.
 */
struct PerceptionParams {
  // ---- Input topics & frame ----
  std::string point_cloud_topic{"/camera/depth_registered/points"};
  std::string rgb_image_topic{"/camera/color/image_raw"};
  std::string target_frame{"base_link"};

  // ---- Processing ----
  double processing_rate{0.5};  ///< How often (Hz) to run the pipeline.
  bool publish_processed_cloud{true};
  bool publish_objects_cloud{true};
  bool publish_support_plane_cloud{true};
  bool publish_object_markers{true};

  // ---- Crop box ----
  bool enable_cropping{true};
  double crop_min_x{0.10};
  double crop_max_x{1.10};
  double crop_min_y{0.00};
  double crop_max_y{0.90};
  double crop_min_z{-std::numeric_limits<double>::infinity()};
  double crop_max_z{std::numeric_limits<double>::infinity()};

  // ---- Plane & object segmentation ----
  int max_iterations{100};
  double distance_threshold{0.01};
  double z_tolerance{0.03};
  double angle_tolerance{0.9990482216};
  double plane_segmentation_threshold{0.001};
  int min_cluster_size{150};
  int max_cluster_size{1000000};
  double cluster_tolerance{0.02};
  int normal_estimation_k{30};
  double w_inliers{1.0};
  double w_size{1.0};
  double w_distance{1.0};
  double w_orientation{1.0};
  int max_plane_segmentation_iterations{1000};
  double plane_segmentation_distance_threshold{0.01};

  // ---- Support surface ----
  std::string support_surface_name{"support_surface"};
  double min_surface_thickness{0.0001};

  // ---- Normal / curvature / RSD ----
  int k_neighbors{30};
  double max_plane_error{0.01};
  int max_iterations_normals{100};
  int min_boundary_neighbors{10};
  double rsd_radius{0.02};

  // ---- Cluster extraction ----
  int nearest_neighbors{30};
  float smoothness_threshold{20.0f};
  float curvature_threshold{0.2f};

  // ---- Object segmentation ----
  int num_iterations{5};
  int inlier_threshold{85};
  int hough_radius_bins{50};
  int hough_center_bins{50};
  double ransac_distance_threshold{0.001};
  int ransac_max_iterations{1000};
  int circle_min_cluster_size{20};
  int circle_max_clusters{2};
  double circle_height_tolerance{0.025};
  double circle_curvature_threshold{0.0015};
  double circle_radius_tolerance{0.020};
  double circle_normal_angle_threshold{0.2};
  double circle_cluster_tolerance{0.025};
  int line_min_cluster_size{20};
  int line_max_clusters{1};
  double line_curvature_threshold{0.0011};
  double line_cluster_tolerance{0.025};
  double line_rho_threshold{0.01};
  double line_theta_threshold{0.1};

  // ---- Legacy shape fitting (used by fitShapeToCluster fallback) ----
  int shape_fitting_max_iterations{1000};
  double shape_fitting_distance_threshold{0.01};
  double shape_fitting_min_radius{0.01};
  double shape_fitting_max_radius{0.1};
  double shape_fitting_normal_distance_weight{0.1};
  double shape_fitting_normal_search_radius{0.05};

  // ---- Target object description (no service request needed) ----
  std::string target_shape{"cylinder"};
  std::vector<double> target_dimensions{0.06, 0.0125};  ///< {height, radius} or {x, y, z}

  // ---- Debug ----
  std::string output_directory{"/tmp/"};
  std::string debug_pcd_filename{"debug_cloud.pcd"};
  bool save_debug_pcd{false};
};

/**
 * @brief Transform a PointCloud2 message into the target frame and optionally
 *        crop it with a box filter.
 *
 * @return Transformed (and optionally cropped) PointCloud2, or nullptr on
 *         failure.
 */
sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
    const std::string& target_frame,
    tf2_ros::Buffer& tf_buffer,
    const PerceptionParams& params,
    const rclcpp::Logger& logger,
    const rclcpp::Time& stamp);

/**
 * @brief Convert a PointCloud2 message to a PCL PointCloud<pcl::PointXYZRGB>.
 * @return Pointer to the PCL cloud, or nullptr on failure.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
    const rclcpp::Logger& logger);

/**
 * @brief Build a box CollisionObject representing the support surface.
 */
moveit_msgs::msg::CollisionObject createSupportSurfaceObject(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_cloud,
    const pcl::ModelCoefficients::Ptr& plane_coefficients,
    const std::string& frame_id,
    const PerceptionParams& params,
    const rclcpp::Logger& logger,
    const rclcpp::Time& stamp);

/**
 * @brief Fit a primitive shape (box or cylinder) to a point cloud cluster.
 *
 * This is the legacy shape-fitting fallback that mirrors
 * GetPlanningSceneServer::fitShapeToCluster.
 */
moveit_msgs::msg::CollisionObject fitShapeToCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster,
    const std::string& frame_id,
    int index,
    const PerceptionParams& params,
    const rclcpp::Logger& logger,
    const rclcpp::Time& stamp);

/**
 * @brief Pick the collision object that best matches the requested target
 *        shape and dimensions.
 * @return The object id of the best match, or empty string if none.
 */
std::string identifyTargetObject(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects,
    const std::string& target_shape,
    const std::vector<double>& target_dimensions,
    const rclcpp::Logger& logger);

/**
 * @brief Build an RViz MarkerArray with one marker per collision object so the
 *        detected scene can be visualised without the planning scene.
 */
visualization_msgs::msg::MarkerArray createObjectMarkers(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects,
    const std::string& target_object_id,
    const rclcpp::Time& stamp);

}  // namespace mycobot_perceptions

#endif  // MYCOBOT_PERCEPTIONS_PERCEPTION_UTILS_HPP
