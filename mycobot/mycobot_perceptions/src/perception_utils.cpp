/**
 * @file perception_utils.cpp
 * @brief Implementation of the perception helpers.
 *
 * The logic here mirrors the private methods of GetPlanningSceneServer in
 * mycobot_mtc_pick_place_demo/src/get_planning_scene_server.cpp, but is
 * refactored into free functions that take an explicit logger and the bundled
 * PerceptionParams struct. This makes the pipeline reusable from any node and
 * keeps the camera_perception node thin.
 *
 * @author hl
 * @date August 2026
 */

#include "mycobot_perceptions/perception_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/exceptions.h>

namespace mycobot_perceptions {

sensor_msgs::msg::PointCloud2::SharedPtr transformPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
    const std::string& target_frame,
    tf2_ros::Buffer& tf_buffer,
    const PerceptionParams& params,
    const rclcpp::Logger& logger,
    const rclcpp::Time& stamp) {
  RCLCPP_INFO(logger, "Transforming point cloud from %s to %s",
              cloud_msg->header.frame_id.c_str(), target_frame.c_str());

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer.lookupTransform(
        target_frame, cloud_msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(logger, "Could not transform point cloud: %s", ex.what());
    return nullptr;
  }

  // Convert ROS PointCloud2 -> PCL PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

  const Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform_stamped);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform_eigen);

  if (params.enable_cropping) {
    RCLCPP_INFO(logger, "Cropping is enabled. Applying crop box filter.");
    pcl::CropBox<pcl::PointXYZRGB> crop_box;
    crop_box.setInputCloud(transformed_cloud);
    crop_box.setMin(Eigen::Vector4f(params.crop_min_x, params.crop_min_y, params.crop_min_z, 1.0));
    crop_box.setMax(Eigen::Vector4f(params.crop_max_x, params.crop_max_y, params.crop_max_z, 1.0));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    crop_box.filter(*cropped_cloud);
    transformed_cloud = cropped_cloud;
    RCLCPP_INFO(logger, "Point cloud cropped. New size: %zu points", transformed_cloud->size());
  } else {
    RCLCPP_INFO(logger, "Cropping is disabled. Using full transformed point cloud.");
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud_out(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*transformed_cloud, *cloud_out);
  cloud_out->header.frame_id = target_frame;
  cloud_out->header.stamp = stamp;

  RCLCPP_INFO(logger, "Point cloud transformed successfully");
  return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg,
    const rclcpp::Logger& logger) {
  RCLCPP_INFO(logger, "Converting PointCloud2 to PCL PointCloud");

  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  try {
    if (!cloud_msg) {
      throw std::runtime_error("Input PointCloud2 message is null");
    }
    if (cloud_msg->data.empty()) {
      throw std::runtime_error("Input PointCloud2 message has no data");
    }
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
    if (pcl_cloud->empty()) {
      throw std::runtime_error("Resulting PCL cloud is empty after conversion");
    }
    RCLCPP_INFO(logger, "PointCloud2 successfully converted to PCL PointCloud with %zu points",
                pcl_cloud->size());
  } catch (const pcl::PCLException& e) {
    RCLCPP_ERROR(logger, "PCL error in convertToPCL: %s", e.what());
    return nullptr;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error in convertToPCL: %s", e.what());
    return nullptr;
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown error occurred in convertToPCL");
    return nullptr;
  }
  return pcl_cloud;
}

moveit_msgs::msg::CollisionObject createSupportSurfaceObject(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_cloud,
    const pcl::ModelCoefficients::Ptr& plane_coefficients,
    const std::string& frame_id,
    const PerceptionParams& params,
    const rclcpp::Logger& logger,
    const rclcpp::Time& stamp) {
  RCLCPP_INFO(logger, "Creating support surface object");

  moveit_msgs::msg::CollisionObject support_surface;
  try {
    if (!plane_coefficients || plane_coefficients->values.size() != 4) {
      throw std::invalid_argument("Invalid plane coefficients");
    }
    if (frame_id.empty()) {
      throw std::invalid_argument("Empty frame_id");
    }
    if (!plane_cloud || plane_cloud->empty()) {
      throw std::invalid_argument("Invalid or empty plane point cloud");
    }

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);

    if (params.enable_cropping) {
      min_pt[0] = std::max(min_pt[0], static_cast<float>(params.crop_min_x));
      min_pt[1] = std::max(min_pt[1], static_cast<float>(params.crop_min_y));
      min_pt[2] = std::max(min_pt[2], static_cast<float>(params.crop_min_z));
      max_pt[0] = std::min(max_pt[0], static_cast<float>(params.crop_max_x));
      max_pt[1] = std::min(max_pt[1], static_cast<float>(params.crop_max_y));
      max_pt[2] = std::min(max_pt[2], static_cast<float>(params.crop_max_z));
    }

    const Eigen::Vector4f centroid = (min_pt + max_pt) / 2.0f;

    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = max_pt[0] - min_pt[0];
    box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = max_pt[1] - min_pt[1];
    box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = max_pt[2] - min_pt[2];
    if (box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] < params.min_surface_thickness) {
      box_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = params.min_surface_thickness;
    }

    geometry_msgs::msg::Pose box_pose;
    Eigen::Vector3d normal(plane_coefficients->values[0], plane_coefficients->values[1],
                           plane_coefficients->values[2]);
    normal.normalize();

    Eigen::Quaterniond rotation;
    rotation.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal);

    box_pose.position.x = centroid[0];
    box_pose.position.y = centroid[1];
    box_pose.position.z = centroid[2];
    box_pose.orientation.x = rotation.x();
    box_pose.orientation.y = rotation.y();
    box_pose.orientation.z = rotation.z();
    box_pose.orientation.w = rotation.w();

    RCLCPP_INFO(logger, "Support surface box dimensions: [%.4f, %.4f, %.4f]",
                box_primitive.dimensions[0], box_primitive.dimensions[1], box_primitive.dimensions[2]);

    support_surface.header.frame_id = frame_id;
    support_surface.header.stamp = stamp;
    support_surface.id = params.support_surface_name;
    support_surface.primitives.push_back(box_primitive);
    support_surface.primitive_poses.push_back(box_pose);
    support_surface.operation = moveit_msgs::msg::CollisionObject::ADD;

    RCLCPP_INFO(logger, "Support surface object created successfully as a box");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Error creating support surface object: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "Unknown error occurred while creating support surface object");
  }
  return support_surface;
}

moveit_msgs::msg::CollisionObject fitShapeToCluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster,
    const std::string& frame_id,
    int index,
    const PerceptionParams& params,
    const rclcpp::Logger& logger,
    const rclcpp::Time& stamp) {
  RCLCPP_INFO(logger, "Fitting shape to cluster %d", index);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.header.stamp = stamp;
  collision_object.id = "object_" + std::to_string(index);

  if (!cluster || cluster->empty()) {
    RCLCPP_WARN(logger, "Empty cluster. Skipping shape fitting.");
    return collision_object;
  }

  struct FitResult {
    std::string shape_type;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;
    double fitness_score{0.0};
  };
  std::vector<FitResult> fit_results;

  // --- Box (parallel plane) fitting ---
  {
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(params.shape_fitting_max_iterations);
    seg.setDistanceThreshold(params.shape_fitting_distance_threshold);

    FitResult box_fit;
    box_fit.shape_type = "box";
    box_fit.coefficients = std::make_shared<pcl::ModelCoefficients>();
    box_fit.inliers = std::make_shared<pcl::PointIndices>();

    seg.setInputCloud(cluster);
    seg.segment(*box_fit.inliers, *box_fit.coefficients);

    if (!box_fit.inliers->indices.empty()) {
      box_fit.fitness_score = static_cast<double>(box_fit.inliers->indices.size()) / cluster->size();
      fit_results.push_back(box_fit);
    }
  }

  // --- Cylinder fitting ---
  {
    auto ne = std::make_shared<pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal>>();
    auto cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    ne->setSearchMethod(tree);
    ne->setInputCloud(cluster);
    ne->setRadiusSearch(params.shape_fitting_normal_search_radius);
    ne->compute(*cloud_normals);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(params.shape_fitting_max_iterations);
    seg.setDistanceThreshold(params.shape_fitting_distance_threshold);
    seg.setRadiusLimits(params.shape_fitting_min_radius, params.shape_fitting_max_radius);
    seg.setNormalDistanceWeight(params.shape_fitting_normal_distance_weight);
    seg.setInputCloud(cluster);
    seg.setInputNormals(cloud_normals);

    FitResult cylinder_fit;
    cylinder_fit.shape_type = "cylinder";
    cylinder_fit.coefficients = std::make_shared<pcl::ModelCoefficients>();
    cylinder_fit.inliers = std::make_shared<pcl::PointIndices>();
    seg.segment(*cylinder_fit.inliers, *cylinder_fit.coefficients);

    if (!cylinder_fit.inliers->indices.empty()) {
      cylinder_fit.fitness_score =
          static_cast<double>(cylinder_fit.inliers->indices.size()) / cluster->size();
      fit_results.push_back(cylinder_fit);
    }
  }

  // --- Pick best fit ---
  FitResult best_fit;
  double best_score = 0.0;
  for (const auto& fit : fit_results) {
    if (fit.fitness_score > best_score) {
      best_score = fit.fitness_score;
      best_fit = fit;
    }
  }

  if (best_fit.shape_type.empty()) {
    RCLCPP_WARN(logger, "No shape could be fitted to the cluster.");
    return collision_object;
  }

  shape_msgs::msg::SolidPrimitive primitive;
  geometry_msgs::msg::Pose pose;

  if (best_fit.shape_type == "box") {
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = max_pt[0] - min_pt[0];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = max_pt[1] - min_pt[1];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = max_pt[2] - min_pt[2];
    pose.position.x = (min_pt[0] + max_pt[0]) / 2;
    pose.position.y = (min_pt[1] + max_pt[1]) / 2;
    pose.position.z = (min_pt[2] + max_pt[2]) / 2;
  } else if (best_fit.shape_type == "cylinder") {
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    primitive.dimensions.resize(2);
    const Eigen::Vector3f axis(best_fit.coefficients->values[3],
                               best_fit.coefficients->values[4],
                               best_fit.coefficients->values[5]);
    const Eigen::Vector3f center(best_fit.coefficients->values[0],
                                 best_fit.coefficients->values[1],
                                 best_fit.coefficients->values[2]);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] =
        best_fit.coefficients->values[6];

    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    const Eigen::Vector3f diff = max_pt.head<3>() - min_pt.head<3>();
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] =
        diff.dot(axis.normalized());

    Eigen::Quaternionf orientation;
    orientation.setFromTwoVectors(Eigen::Vector3f::UnitZ(), axis);
    pose.position.x = center[0];
    pose.position.y = center[1];
    pose.position.z = center[2];
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
  }

  collision_object.id = best_fit.shape_type + "_" + std::to_string(index);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(logger, "Fitted %s to cluster %d with score %.2f",
              best_fit.shape_type.c_str(), index, best_score);
  return collision_object;
}

std::string identifyTargetObject(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects,
    const std::string& target_shape,
    const std::vector<double>& target_dimensions,
    const rclcpp::Logger& logger) {
  double best_score = 0.0;
  std::string best_match_id;

  for (const auto& object : objects) {
    if (object.primitives.empty()) continue;
    const auto& primitive = object.primitives[0];
    double shape_score = 0.0;
    double dimension_score = 0.0;

    if ((target_shape == "cylinder" && primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER) ||
        (target_shape == "box" && primitive.type == shape_msgs::msg::SolidPrimitive::BOX)) {
      shape_score = 1.0;
    } else {
      continue;
    }

    std::vector<double> object_dimensions;
    switch (primitive.type) {
      case shape_msgs::msg::SolidPrimitive::CYLINDER:
        object_dimensions = {primitive.dimensions[primitive.CYLINDER_HEIGHT],
                             primitive.dimensions[primitive.CYLINDER_RADIUS]};
        break;
      case shape_msgs::msg::SolidPrimitive::BOX:
        object_dimensions = {primitive.dimensions[primitive.BOX_X],
                             primitive.dimensions[primitive.BOX_Y],
                             primitive.dimensions[primitive.BOX_Z]};
        break;
      default:
        continue;
    }

    if (object_dimensions.size() == target_dimensions.size()) {
      double total_diff = 0.0;
      for (size_t i = 0; i < object_dimensions.size(); ++i) {
        const double diff = std::abs(object_dimensions[i] - target_dimensions[i]);
        total_diff += diff / std::max(target_dimensions[i], 1e-9);
      }
      dimension_score = std::max(0.0, 1.0 - (total_diff / object_dimensions.size()));
    }

    const double similarity_score = 0.7 * shape_score + 0.3 * dimension_score;
    if (similarity_score > best_score) {
      best_score = similarity_score;
      best_match_id = object.id;
    }
  }

  if (!best_match_id.empty()) {
    RCLCPP_INFO(logger, "Best matching object found: %s with similarity score: %.2f",
                best_match_id.c_str(), best_score);
  } else {
    RCLCPP_WARN(logger, "No matching object found for the target shape and dimensions");
  }
  return best_match_id;
}

visualization_msgs::msg::MarkerArray createObjectMarkers(
    const std::vector<moveit_msgs::msg::CollisionObject>& objects,
    const std::string& target_object_id,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;
  for (const auto& obj : objects) {
    if (obj.primitives.empty() || obj.primitive_poses.empty()) continue;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = obj.header.frame_id;
    marker.header.stamp = stamp;
    marker.ns = "perceived_objects";
    marker.id = marker_id++;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = obj.primitive_poses[0];
    // lifetime left at default (zero) => persistent until replaced

    const auto& prim = obj.primitives[0];
    const bool is_target = (!target_object_id.empty() && obj.id == target_object_id);
    // Highlight the target in green, others in grey.
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 0.6f;
    if (is_target) {
      marker.color.r = 0.1f;
      marker.color.g = 0.9f;
      marker.color.b = 0.1f;
      marker.color.a = 0.8f;
    }

    switch (prim.type) {
      case shape_msgs::msg::SolidPrimitive::BOX:
        marker.type = visualization_msgs::msg::Marker::CUBE;
        if (prim.dimensions.size() >= 3) {
          marker.scale.x = prim.dimensions[prim.BOX_X];
          marker.scale.y = prim.dimensions[prim.BOX_Y];
          marker.scale.z = prim.dimensions[prim.BOX_Z];
        }
        break;
      case shape_msgs::msg::SolidPrimitive::CYLINDER:
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        if (prim.dimensions.size() >= 2) {
          marker.scale.x = 2.0 * prim.dimensions[prim.CYLINDER_RADIUS];
          marker.scale.y = 2.0 * prim.dimensions[prim.CYLINDER_RADIUS];
          marker.scale.z = prim.dimensions[prim.CYLINDER_HEIGHT];
        }
        break;
      default:
        continue;
    }
    // Ensure non-zero scales (rviz otherwise warns).
    marker.scale.x = std::max(marker.scale.x, 0.001);
    marker.scale.y = std::max(marker.scale.y, 0.001);
    marker.scale.z = std::max(marker.scale.z, 0.001);

    markers.markers.push_back(marker);

    // Text label
    visualization_msgs::msg::Marker text_marker = marker;
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.scale.x = 0.05;
    text_marker.scale.y = 0.05;
    text_marker.scale.z = 0.05;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.b = 1.0f;
    text_marker.color.a = 1.0f;
    text_marker.pose.position.z += (marker.scale.z / 2.0) + 0.03;
    text_marker.text = obj.id + (is_target ? " (target)" : "");
    markers.markers.push_back(text_marker);
  }
  return markers;
}

}  // namespace mycobot_perceptions
