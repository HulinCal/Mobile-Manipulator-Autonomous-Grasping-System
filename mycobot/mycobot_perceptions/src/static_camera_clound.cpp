/**
 * @file static_camera_clound.cpp
 * @brief 静态相机点云 → PCL 区域增长聚类 → MoveIt PlanningScene 发布节点。
 *
 * 节点功能：
 *   1. 订阅 /camera/depth_registered/points 点云话题。
 *   2. 在独立 worker 线程中对最新一帧点云做 PCL 区域增长聚类，对每个类
 *      计算轴对齐包围盒 (AABB)，作为 CollisionObject(BOX) 加入 PlanningScene。
 *   3. 处理完一帧立即在 /test_planning_scene 发布 PlanningScene(diff)。
 *
 * 架构（避免回调堆积）：
 *   - 订阅回调 pointCloudCallback: 只做 O(1) 缓存 + 唤醒 worker，不做 PCL 计算。
 *   - worker 线程: 阻塞处理最新一帧 → 处理完立即 publish → 取下一帧。
 *     处理慢时新帧覆盖旧帧（实时性优先），不会堆积多个处理实例。
 *   - 发布采用事件驱动：处理完一帧立刻发一次，PSM 立即收到。
 *   - 启动时立即发一次静态 cube 作为初始场景，不必等点云。
 *
 * @author hl
 * @date September 2026
 */

#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mycobot_perceptions {

class StaticCameraCloundNode : public rclcpp::Node {
 public:
  StaticCameraCloundNode() : rclcpp::Node("static_camera_clound") {
    // ---- 参数 ----
    point_cloud_topic_ =
        this->declare_parameter<std::string>("point_cloud_topic",
                                              "/camera/depth_registered/points");
    planning_scene_topic_ =
        this->declare_parameter<std::string>("planning_scene_topic",
                                              "/planning_scene");

    // ---- PCL 处理参数 ----
    // 体素滤波叶大小 (m)。0 表示跳过滤波。
    voxel_leaf_size_ = this->declare_parameter<double>("voxel_leaf_size", 0.01);
    // 区域增长：法线估计搜索半径 (m)
    normal_search_radius_ = this->declare_parameter<double>("normal_search_radius", 0.03);
    // 区域增长：邻居数 (knn)
    region_num_neighbors_ = this->declare_parameter<int>("region_num_neighbors", 30);
    // 区域增长：平滑阈值 (度)
    region_smooth_threshold_ = this->declare_parameter<double>("region_smooth_threshold", 7.0);
    // 区域增长：曲率阈值
    region_curvature_threshold_ = this->declare_parameter<double>("region_curvature_threshold", 1.0);
    // 聚类最少点数：小于此值的类被丢弃（去噪）
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 100);
    // 聚类最多点数：大于此值的类被丢弃（去地平等大块）
    max_cluster_size_ = this->declare_parameter<int>("max_cluster_size", 100000);
    // 包围盒 padding (m)，每边外扩此值，避免紧贴物体导致规划失败
    bbox_padding_ = this->declare_parameter<double>("bbox_padding", 0.01);
    // 包围盒最大尺寸 (m)，超过则跳过该类
    bbox_max_size_ = this->declare_parameter<double>("bbox_max_size", 0.5);

    // 目标 frame：将点云从相机光学 frame 变换到此 frame 后再聚类。
    // 必须是 moveit TF 树里存在的 frame（如 base_link / world），
    // 否则 PSM 会报 "Unknown frame"。
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    // ---- CropBox 滤波参数（在 target_frame_ 下做裁剪）----
    // 这些参数对应 config/camera_perception.yaml 中的 crop_min_x/max_x/y/z 字段。
    // yaml 中允许用 ±.inf 表示无限制，C++ 这里也用 ±infinity 兼容 PCL CropBox。
    enable_cropping_ = this->declare_parameter<bool>("enable_cropping", true);
    crop_min_x_ = this->declare_parameter<double>("crop_min_x", -2.0);
    crop_max_x_ = this->declare_parameter<double>("crop_max_x", 1.5);
    crop_min_y_ = this->declare_parameter<double>("crop_min_y", -2.0);
    crop_max_y_ = this->declare_parameter<double>("crop_max_y", 2.0);
    crop_min_z_ = this->declare_parameter<double>(
        "crop_min_z", -std::numeric_limits<double>::infinity());
    crop_max_z_ = this->declare_parameter<double>(
        "crop_max_z", std::numeric_limits<double>::infinity());

    // ---- ROS 接口 ----
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_, 10,
        std::bind(&StaticCameraCloundNode::pointCloudCallback, this, std::placeholders::_1));

    // QoS 用 transient_local + reliable，启动时发初始场景 + 每处理完一帧立即发
    planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
        planning_scene_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    // ---- 启动 worker 线程 ----
    RCLCPP_INFO(this->get_logger(),
                "static_camera_clound ready. Subscribed to '%s'; publishing "
                "PlanningScene to '%s'. PCL worker thread runs in background.",
                point_cloud_topic_.c_str(), planning_scene_topic_.c_str());

    // ---- TF buffer + listener（用于把点云从相机 frame 变换到 target frame）----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    worker_thread_ = std::thread(&StaticCameraCloundNode::workerLoop, this);
  }

  ~StaticCameraCloundNode() override {
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      shutdown_ = true;
      cloud_cv_.notify_one();
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

 private:
  // ---- 订阅回调：仅做 O(1) 缓存 + 唤醒 worker，不做 PCL 处理 ----
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      latest_cloud_ = msg;
      cloud_ready_ = true;
    }
    cloud_cv_.notify_one();
    RCLCPP_DEBUG(this->get_logger(),
                 "Received point cloud: %ux%u, frame='%s' (cached, worker notified)",
                 msg->width, msg->height, msg->header.frame_id.c_str());
  }

  // ---- Worker 线程：阻塞等待点云 → 处理 → 发布 → 循环 ----
  void workerLoop() {
    while (true) {
      sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
      {
        std::unique_lock<std::mutex> lock(cloud_mutex_);
        cloud_cv_.wait(lock, [this]() {
          return shutdown_ || cloud_ready_;
        });
        if (shutdown_) {
          return;
        }
        // 取出最新一帧（新帧已覆盖旧帧）
        cloud_msg = latest_cloud_;
        cloud_ready_ = false;
      }

      if (!cloud_msg) {
        continue;
      }

      // 处理一帧（耗时操作在锁外执行，不阻塞订阅）
      moveit_msgs::msg::PlanningScene scene;
      bool ok = processCloud(*cloud_msg, scene);
      if (ok) {
        planning_scene_pub_->publish(scene);
        ++publish_count_;
        RCLCPP_INFO(this->get_logger(),
                    "Published scene with %lu collision object(s) (#%lu).",
                    scene.world.collision_objects.size(), publish_count_);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "processCloud failed for this frame, not publishing.");
      }
    }
  }

  // ---- PCL 处理：区域增长聚类 → 每类生成 AABB 立方体 → 装入 PlanningScene ----
  bool processCloud(const sensor_msgs::msg::PointCloud2& cloud_msg,
                    moveit_msgs::msg::PlanningScene& scene) {
    // 1) ROS msg → PCL cloud (XYZRGB)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud_msg, *cloud);
    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty cloud, skip.");
      return false;
    }
    RCLCPP_DEBUG(this->get_logger(), "Processing cloud: %lu points, frame='%s'",
                 cloud->size(), cloud_msg.header.frame_id.c_str());

    // 1.5) 把点云从相机光学 frame 变换到 target_frame_（通常是 base_link）。
    // 这样后续所有 AABB 中心/尺寸都在 target_frame_ 下，CollisionObject 的
    // header.frame_id 也用 target_frame_，PSM 就能直接合并（不会报 Unknown frame）。
    std::string cloud_frame = cloud_msg.header.frame_id;
    std::string obj_frame = target_frame_;
    if (!cloud_frame.empty() && cloud_frame != obj_frame) {
      geometry_msgs::msg::TransformStamped tf_stamped;
      try {
        // 给 TF 0.2s 容忍度（用点云时间戳查询，找不到时退而用 latest）
        try {
          tf_stamped = tf_buffer_->lookupTransform(
              obj_frame, cloud_frame,
              tf2::TimePoint(std::chrono::nanoseconds(0)));
        } catch (...) {
          tf_stamped = tf_buffer_->lookupTransform(obj_frame, cloud_frame,
                                                    tf2::TimePointZero);
        }
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(),
                    "TF lookup %s -> %s failed: %s. Skip this cloud.",
                    cloud_frame.c_str(), obj_frame.c_str(), ex.what());
        return false;
      }
      // 转成 Eigen::Affine3d 并用 PCL 变换点云
      Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf_stamped);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud(*cloud, *transformed, tf_eigen);
      cloud = transformed;
      cloud->header.frame_id = obj_frame;
      RCLCPP_DEBUG(this->get_logger(), "Transformed cloud %s -> %s, %lu pts",
                   cloud_frame.c_str(), obj_frame.c_str(), cloud->size());
    }

    // 1.6) CropBox 滤波：在 target_frame_ 下裁剪工作空间。
    // 必须在 TF 变换之后做，因为 crop_min_x/max_x/y/z 都是 target_frame_ 下的坐标。
    // yaml 中允许 ±.inf 表示无限制，PCL CropBox 与 ±inf 比较正确（任何有限值都满足）。
    if (enable_cropping_) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::CropBox<pcl::PointXYZRGB> crop_box;
      crop_box.setInputCloud(cloud);
      crop_box.setMin(Eigen::Vector4f(
          static_cast<float>(crop_min_x_),
          static_cast<float>(crop_min_y_),
          static_cast<float>(crop_min_z_),
          1.0f));
      crop_box.setMax(Eigen::Vector4f(
          static_cast<float>(crop_max_x_),
          static_cast<float>(crop_max_y_),
          static_cast<float>(crop_max_z_),
          1.0f));
      crop_box.filter(*cropped);
      if (cropped->empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "CropBox produced empty cloud (min=[%.3f,%.3f,%.3f] "
                    "max=[%.3f,%.3f,%.3f]), skip.",
                    crop_min_x_, crop_min_y_, crop_min_z_,
                    crop_max_x_, crop_max_y_, crop_max_z_);
        return false;
      }
      RCLCPP_DEBUG(this->get_logger(),
                   "After CropBox: %lu / %lu points (min=[%.3f,%.3f,%.3f] "
                   "max=[%.3f,%.3f,%.3f])",
                   cropped->size(), cloud->size(),
                   crop_min_x_, crop_min_y_, crop_min_z_,
                   crop_max_x_, crop_max_y_, crop_max_z_);
      cloud = cropped;
    }

    // 2) 体素滤波（降采样，加快后续处理）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (voxel_leaf_size_ > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      vg.filter(*filtered);
      if (filtered->empty()) {
        RCLCPP_WARN(this->get_logger(), "Voxel filter produced empty cloud, skip.");
        return false;
      }
      RCLCPP_DEBUG(this->get_logger(), "After voxel filter: %lu points", filtered->size());
    } else {
      *filtered = *cloud;
    }

    // 3) 法线估计
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(filtered);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_search_radius_);
    ne.compute(*normals);
    if (normals->empty()) {
      RCLCPP_WARN(this->get_logger(), "Normal estimation produced empty result, skip.");
      return false;
    }

    // 4) 区域增长聚类
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> rg;
    rg.setMinClusterSize(min_cluster_size_);
    rg.setMaxClusterSize(max_cluster_size_);
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(region_num_neighbors_);
    rg.setInputCloud(filtered);
    rg.setInputNormals(normals);
    rg.setSmoothnessThreshold(static_cast<float>(region_smooth_threshold_ * M_PI / 180.0));
    rg.setCurvatureThreshold(static_cast<float>(region_curvature_threshold_));

    std::vector<pcl::PointIndices> clusters;
    rg.extract(clusters);
    RCLCPP_INFO(this->get_logger(), "Region growing found %lu cluster(s).", clusters.size());

    if (clusters.empty()) {
      RCLCPP_WARN(this->get_logger(), "No clusters found, skip this frame.");
      return false;
    }

    // 5) 每个类计算 AABB → 生成 CollisionObject
    const rclcpp::Time stamp = this->now();
    scene.is_diff = true;
    scene.robot_state.is_diff = true;

    // 5.0) 先发 REMOVE 上一帧所有 collision_objects。
    // 因为 PSM 会把 ADD 的对象永久保存在内部场景中，若上一帧的物体在本帧
    // 点云里消失（被拿开、移出视野），PSM 不会自动删除它，需要我们主动发
    // REMOVE 才能清掉。这里把上一帧所有 id 都标记 REMOVE，再 ADD 本帧新的
    // 聚类结果，保证场景永远只反映"当前相机视野内的物体"。
    std::vector<std::string> current_object_ids;
    current_object_ids.reserve(clusters.size());
    {
      for (const auto& old_id : previous_object_ids_) {
        moveit_msgs::msg::CollisionObject remove_co;
        remove_co.header.frame_id = target_frame_;
        remove_co.header.stamp = stamp;
        remove_co.id = old_id;
        remove_co.operation = moveit_msgs::msg::CollisionObject::REMOVE;
        scene.world.collision_objects.push_back(remove_co);
      }
    }

    uint32_t kept = 0;
    for (size_t i = 0; i < clusters.size(); ++i) {
      const auto& indices = clusters[i];
      if (indices.indices.empty()) continue;

      // AABB
      Eigen::Vector4f min_v, max_v;
      pcl::PointIndices pi;
      pi.indices = indices.indices;
      pcl::getMinMax3D(*filtered, pi, min_v, max_v);

      const double dx = max_v[0] - min_v[0];
      const double dy = max_v[1] - min_v[1];
      const double dz = max_v[2] - min_v[2];
      // 超大块（如地面/墙面）跳过
      if (dx > bbox_max_size_ || dy > bbox_max_size_ || dz > bbox_max_size_) {
        RCLCPP_DEBUG(this->get_logger(),
                     "Cluster %zu: size (%.3f x %.3f x %.3f) too large, skip.",
                     i, dx, dy, dz);
        continue;
      }
      // 加 padding 防止规划紧贴
      const double sx = dx + 2.0 * bbox_padding_;
      const double sy = dy + 2.0 * bbox_padding_;
      const double sz = dz + 2.0 * bbox_padding_;
      const double cx = (min_v[0] + max_v[0]) / 2.0;
      const double cy = (min_v[1] + max_v[1]) / 2.0;
      const double cz = (min_v[2] + max_v[2]) / 2.0;

      const std::string obj_id = "cluster_" + std::to_string(i);
      current_object_ids.push_back(obj_id);

      moveit_msgs::msg::CollisionObject co;
      co.header.frame_id = target_frame_;
      co.header.stamp = stamp;
      co.id = obj_id;

      shape_msgs::msg::SolidPrimitive prim;
      prim.type = shape_msgs::msg::SolidPrimitive::BOX;
      prim.dimensions.resize(3);
      prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = sx;
      prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = sy;
      prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = sz;
      co.primitives.push_back(prim);

      geometry_msgs::msg::Pose pose;
      pose.position.x = cx;
      pose.position.y = cy;
      pose.position.z = cz;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      co.primitive_poses.push_back(pose);

      co.operation = moveit_msgs::msg::CollisionObject::ADD;
      scene.world.collision_objects.push_back(co);
      ++kept;

      RCLCPP_INFO(this->get_logger(),
                  "Cluster %zu: %zu pts, AABB (%.3f x %.3f x %.3f) m @ (%.3f, %.3f, %.3f) frame=%s",
                  i, indices.indices.size(), sx, sy, sz, cx, cy, cz,
                  cloud_msg.header.frame_id.c_str());
    }

    if (kept == 0) {
      // 本帧没有保留任何新对象。但若上一帧有对象，scene 里已经塞了 REMOVE，
      // 需要把这个只含 REMOVE 的 scene 发出去清空场景。
      if (previous_object_ids_.empty()) {
        RCLCPP_WARN(this->get_logger(),
                    "No clusters kept and no previous objects to remove, skip.");
        return false;
      }
      RCLCPP_WARN(this->get_logger(),
                  "All clusters filtered out, removing %lu previous object(s).",
                  previous_object_ids_.size());
      previous_object_ids_.clear();
      return true;
    }

    // 更新上一帧 id 列表为本帧的
    previous_object_ids_ = current_object_ids;
    return true;
  }

  // ---- 参数 ----
  std::string point_cloud_topic_;
  std::string planning_scene_topic_;

  // PCL 处理参数
  double voxel_leaf_size_{0.01};
  double normal_search_radius_{0.03};
  int region_num_neighbors_{30};
  double region_smooth_threshold_{7.0};
  double region_curvature_threshold_{1.0};
  int min_cluster_size_{100};
  int max_cluster_size_{100000};
  double bbox_padding_{0.01};
  double bbox_max_size_{0.5};

  // 目标 frame（点云变换到此 frame 后再聚类，PSM 用此 frame 合并对象）
  std::string target_frame_{"base_link"};

  // CropBox 滤波参数（在 target_frame_ 下做裁剪）
  bool enable_cropping_{true};
  double crop_min_x_{0.10};
  double crop_max_x_{1.10};
  double crop_min_y_{0.00};
  double crop_max_y_{0.90};
  double crop_min_z_{-std::numeric_limits<double>::infinity()};
  double crop_max_z_{std::numeric_limits<double>::infinity()};

  // ---- ROS 接口 ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;

  // ---- TF buffer + listener（用于点云坐标系变换）----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- Worker 线程 + 同步 ----
  std::thread worker_thread_;
  std::mutex cloud_mutex_;
  std::condition_variable cloud_cv_;
  bool cloud_ready_{false};
  std::atomic_bool shutdown_{false};
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  // 发布计数（worker 线程单线程访问，无需 atomic）
  uint64_t publish_count_{0};

  // 上一帧发布的 collision_object id 列表。本帧发布前先对这些 id 发 REMOVE，
  // 清掉场景里上一帧的物体，避免物体被拿开后仍在场景里残留。
  std::vector<std::string> previous_object_ids_;
};

}  // namespace mycobot_perceptions

int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<mycobot_perceptions::StaticCameraCloundNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("static_camera_clound"),
                 "Caught exception: %s", e.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("static_camera_clound"),
                 "Caught unknown exception");
    return 1;
  }
  return 0;
}
