// 定位异常监控节点：
// 1) 监控 AMCL 位姿协方差，判断定位是否发散；
// 2) 用激光扫描与静态地图做一致性校验，判断是否被"绑架"到别处；
// 3) 异常持续达到阈值且小车静止时，自动触发 /initialize_localization。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wheeltec_nav2/action/initialize_localization.hpp"

namespace wheeltec_nav2
{

class LocalizationMonitor : public rclcpp::Node
{
public:
  using InitializeLocalization = wheeltec_nav2::action::InitializeLocalization;
  using GoalHandle = rclcpp_action::ClientGoalHandle<InitializeLocalization>;
  using SteadyClock = std::chrono::steady_clock;

  LocalizationMonitor()
  : Node("localization_monitor")
  {
    loadParameters();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    action_client_ = rclcpp_action::create_client<InitializeLocalization>(
      this, initialize_action_);

    // /amcl_pose 为 TRANSIENT_LOCAL，/map 为 TRANSIENT_LOCAL，必须显式匹配 QoS
    rclcpp::QoS amcl_qos(rclcpp::KeepLast(10));
    amcl_qos.transient_local().reliable();
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local().reliable();

    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      amcl_pose_topic_, amcl_qos,
      std::bind(&LocalizationMonitor::onAmclPose, this, std::placeholders::_1));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&LocalizationMonitor::onMap, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LocalizationMonitor::onScan, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&LocalizationMonitor::onOdom, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LocalizationMonitor::evaluate, this));

    start_time_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(),
      "定位异常监控已启动：动作=%s，协方差阈值(%.2fm/%.2frad)，扫描阈值=%.2f（至少 %d 束失配/ %d 扇区）",
      initialize_action_.c_str(), std_xy_threshold_, std_yaw_threshold_,
      scan_mismatch_threshold_, scan_check_min_mismatch_beams_, scan_min_mismatch_sectors_);
  }

private:
  // ── 参数 ──
  std::string initialize_action_;
  std::string map_topic_;
  std::string amcl_pose_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string base_frame_;

  double std_xy_threshold_{0.30};
  double std_yaw_threshold_{0.20};
  double scan_mismatch_threshold_{0.60};
  double scan_match_tolerance_{0.50};
  int scan_check_max_beams_{80};
  int scan_check_min_beams_{40};
  int scan_check_min_mismatch_beams_{25};
  int scan_min_mismatch_sectors_{5};
  double scan_baseline_margin_{0.20};
  double scan_check_max_range_{6.0};
  int occupied_threshold_{65};
  double cov_suppress_move_dist_{0.05};

  double anomaly_sustain_time_{3.0};
  double startup_grace_period_{15.0};
  double recovery_cooldown_{60.0};
  double recovery_timeout_{120.0};
  int max_consecutive_recoveries_{3};
  double failure_hold_time_{120.0};
  double settle_after_success_{5.0};
  bool require_stationary_{true};
  double stationary_linear_threshold_{0.02};
  double stationary_angular_threshold_{0.05};
  double odom_timeout_{1.0};

  void loadParameters()
  {
    initialize_action_ = declare_parameter<std::string>(
      "initialize_action", "/initialize_localization");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_combined");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    std_xy_threshold_ = declare_parameter<double>("std_xy_threshold", std_xy_threshold_);
    std_yaw_threshold_ = declare_parameter<double>("std_yaw_threshold", std_yaw_threshold_);
    scan_mismatch_threshold_ = declare_parameter<double>(
      "scan_mismatch_threshold", scan_mismatch_threshold_);
    scan_match_tolerance_ = declare_parameter<double>(
      "scan_match_tolerance", scan_match_tolerance_);
    scan_check_max_beams_ = declare_parameter<int>(
      "scan_check_max_beams", scan_check_max_beams_);
    scan_check_min_beams_ = declare_parameter<int>(
      "scan_check_min_beams", scan_check_min_beams_);
    scan_check_min_mismatch_beams_ = declare_parameter<int>(
      "scan_check_min_mismatch_beams", scan_check_min_mismatch_beams_);
    scan_min_mismatch_sectors_ = declare_parameter<int>(
      "scan_min_mismatch_sectors", scan_min_mismatch_sectors_);
    scan_baseline_margin_ = declare_parameter<double>(
      "scan_baseline_margin", scan_baseline_margin_);
    scan_check_max_range_ = declare_parameter<double>(
      "scan_check_max_range", scan_check_max_range_);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", occupied_threshold_);
    cov_suppress_move_dist_ = declare_parameter<double>(
      "cov_suppress_move_dist", cov_suppress_move_dist_);

    anomaly_sustain_time_ = declare_parameter<double>(
      "anomaly_sustain_time", anomaly_sustain_time_);
    startup_grace_period_ = declare_parameter<double>(
      "startup_grace_period", startup_grace_period_);
    recovery_cooldown_ = declare_parameter<double>("recovery_cooldown", recovery_cooldown_);
    recovery_timeout_ = declare_parameter<double>("recovery_timeout", recovery_timeout_);
    max_consecutive_recoveries_ = declare_parameter<int>(
      "max_consecutive_recoveries", max_consecutive_recoveries_);
    failure_hold_time_ = declare_parameter<double>("failure_hold_time", failure_hold_time_);
    settle_after_success_ = declare_parameter<double>(
      "settle_after_success", settle_after_success_);
    require_stationary_ = declare_parameter<bool>("require_stationary", require_stationary_);
    stationary_linear_threshold_ = declare_parameter<double>(
      "stationary_linear_threshold", stationary_linear_threshold_);
    stationary_angular_threshold_ = declare_parameter<double>(
      "stationary_angular_threshold", stationary_angular_threshold_);
    odom_timeout_ = declare_parameter<double>("odom_timeout", odom_timeout_);
  }

  // ── 话题回调 ──
  void onMap(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    RCLCPP_INFO(
      get_logger(), "收到地图：%dx%d，分辨率 %.3fm",
      msg->info.width, msg->info.height, msg->info.resolution);
  }

  void onAmclPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    amcl_x_ = msg->pose.pose.position.x;
    amcl_y_ = msg->pose.pose.position.y;
    amcl_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    amcl_cov_ = msg->pose.covariance;
    have_amcl_ = true;
  }

  void onOdom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_vx_ = msg->twist.twist.linear.x;
    odom_vy_ = msg->twist.twist.linear.y;
    odom_wz_ = msg->twist.twist.angular.z;
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_time_ = SteadyClock::now();
    have_odom_ = true;

    // 重定位后 AMCL 需要里程计运动才会更新滤波器，静止时协方差不会收敛，
    // 在产生足够位移前不把协方差作为异常依据，避免静止误报
    if (cov_suppressed_ &&
      std::hypot(odom_x_ - cov_suppress_ref_x_, odom_y_ - cov_suppress_ref_y_) >
      cov_suppress_move_dist_)
    {
      cov_suppressed_ = false;
    }
  }

  void onScan(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_received_ = true;
    if (!map_ || !have_amcl_) {
      return;
    }

    double fraction = 0.0;
    int valid_beams = 0;
    int mismatch_beams = 0;
    int mismatch_sectors = 0;
    if (!computeScanMismatch(
        *msg, fraction, valid_beams, mismatch_beams, mismatch_sectors))
    {
      return;
    }

    scan_mismatch_fraction_ = fraction;
    scan_valid_beams_ = valid_beams;
    scan_mismatch_beams_ = mismatch_beams;
    scan_mismatch_sectors_ = mismatch_sectors;
    scan_mismatch_time_ = SteadyClock::now();

    // 启动稳定期内采集环境基线：地图与现场存在固有差异时，
    // 用基线抬高触发阈值，避免静止状态下的持续误报
    if (!baseline_ready_ && isStationary() &&
      secondsSince(start_time_) < startup_grace_period_)
    {
      baseline_samples_.push_back(fraction);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "激光-地图匹配：失配 %.2f（%d/%d 束，%d 个扇区），触发阈值 %.2f",
      fraction, mismatch_beams, valid_beams, mismatch_sectors, effectiveScanThreshold());
  }

  // ── 异常判定 ──
  bool isCovarianceBad() const
  {
    if (!have_amcl_ || cov_suppressed_) {
      return false;
    }
    const double std_x = std::sqrt(std::max(0.0, amcl_cov_[0]));
    const double std_y = std::sqrt(std::max(0.0, amcl_cov_[7]));
    const double std_yaw = std::sqrt(std::max(0.0, amcl_cov_[35]));
    return std::max(std_x, std_y) > std_xy_threshold_ || std_yaw > std_yaw_threshold_;
  }

  bool isScanMismatchBad() const
  {
    if (!scan_received_) {
      return false;
    }
    if (secondsSince(scan_mismatch_time_) > 1.5) {
      return false;
    }
    return scan_mismatch_fraction_ >= effectiveScanThreshold() &&
           scan_valid_beams_ >= scan_check_min_beams_ &&
           scan_mismatch_beams_ >= scan_check_min_mismatch_beams_ &&
           scan_mismatch_sectors_ >= scan_min_mismatch_sectors_;
  }

  double effectiveScanThreshold() const
  {
    return std::max(
      scan_mismatch_threshold_, scan_baseline_fraction_ + scan_baseline_margin_);
  }

  bool isStationary() const
  {
    if (!have_odom_ || secondsSince(odom_time_) > odom_timeout_) {
      return false;
    }
    return std::hypot(odom_vx_, odom_vy_) < stationary_linear_threshold_ &&
           std::fabs(odom_wz_) < stationary_angular_threshold_;
  }

  // ── 激光与地图一致性校验 ──
  bool computeScanMismatch(
    const sensor_msgs::msg::LaserScan & scan, double & fraction, int & valid_beams,
    int & mismatch_beams, int & mismatch_sectors)
  {
    if (scan.ranges.empty() || map_->data.empty()) {
      return false;
    }

    geometry_msgs::msg::TransformStamped base_to_laser;
    try {
      base_to_laser = tf_buffer_->lookupTransform(
        base_frame_, scan.header.frame_id, tf2::TimePointZero,
        std::chrono::milliseconds(50));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000, "查询 %s->%s 失败：%s",
        base_frame_.c_str(), scan.header.frame_id.c_str(), ex.what());
      return false;
    }

    // 激光原点在 map 系下的位姿 = AMCL 位姿 × 雷达到基座变换
    const auto & t = base_to_laser.transform.translation;
    const double laser_x = amcl_x_ + std::cos(amcl_yaw_) * t.x - std::sin(amcl_yaw_) * t.y;
    const double laser_y = amcl_y_ + std::sin(amcl_yaw_) * t.x + std::cos(amcl_yaw_) * t.y;
    const double laser_yaw = amcl_yaw_ + yawFromQuaternion(base_to_laser.transform.rotation);

    // 雷达原点必须位于已知空闲栅格，否则地图没有可用的比对信息
    int gx0 = 0;
    int gy0 = 0;
    if (!worldToGrid(laser_x, laser_y, gx0, gy0)) {
      return false;
    }
    const int8_t origin_value =
      map_->data[static_cast<size_t>(gy0) * map_->info.width + gx0];
    if (origin_value < 0 || origin_value >= occupied_threshold_) {
      return false;
    }

    const double range_max = std::isfinite(scan.range_max) ?
      std::min(scan_check_max_range_, static_cast<double>(scan.range_max)) :
      scan_check_max_range_;
    const int beam_step = std::max(
      1, static_cast<int>(scan.ranges.size()) / std::max(1, scan_check_max_beams_));

    constexpr int kSectorCount = 8;
    std::array<int, kSectorCount> sector_mismatch{};

    int valid = 0;
    int mismatch = 0;
    for (size_t i = 0; i < scan.ranges.size(); i += beam_step) {
      const double actual = scan.ranges[i];
      const bool actual_valid =
        std::isfinite(actual) && actual >= scan.range_min && actual < scan.range_max - 1e-3;
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double expected = raycast(laser_x, laser_y, laser_yaw + angle, range_max);
      if (expected == kRayUnknown) {
        // 射线经过未知区域或射出地图，地图没有可用信息，跳过该光束
        continue;
      }

      bool is_mismatch = false;
      if (expected == kRayNoHit) {
        // 地图该方向已知空闲：只有雷达确实看到障碍才算不一致
        if (!actual_valid) {
          continue;
        }
        is_mismatch = actual < range_max * 0.9;
      } else {
        // 地图该方向有墙：无回波多为玻璃/镜面/低反射，跳过避免误报
        if (!actual_valid) {
          continue;
        }
        is_mismatch = std::fabs(expected - actual) > scan_match_tolerance_;
      }

      ++valid;
      if (!is_mismatch) {
        continue;
      }
      ++mismatch;

      const double normalized =
        (angle - scan.angle_min) / (scan.angle_max - scan.angle_min + 1e-9);
      const int sector = std::min(
        kSectorCount - 1, std::max(0, static_cast<int>(normalized * kSectorCount)));
      ++sector_mismatch[static_cast<size_t>(sector)];
    }

    if (valid < scan_check_min_beams_) {
      return false;
    }

    int sectors = 0;
    for (const int count : sector_mismatch) {
      if (count >= 2) {
        ++sectors;
      }
    }

    fraction = static_cast<double>(mismatch) / static_cast<double>(valid);
    valid_beams = valid;
    mismatch_beams = mismatch;
    mismatch_sectors = sectors;
    return true;
  }

  bool worldToGrid(double x, double y, int & gx, int & gy) const
  {
    const auto & info = map_->info;
    if (info.resolution <= 0.0) {
      return false;
    }
    const double origin_yaw = yawFromQuaternion(info.origin.orientation);
    const double cos_o = std::cos(-origin_yaw);
    const double sin_o = std::sin(-origin_yaw);
    const double dx = x - info.origin.position.x;
    const double dy = y - info.origin.position.y;
    gx = static_cast<int>(std::floor((dx * cos_o - dy * sin_o) / info.resolution));
    gy = static_cast<int>(std::floor((dx * sin_o + dy * cos_o) / info.resolution));
    return gx >= 0 && gy >= 0 &&
           gx < static_cast<int>(info.width) && gy < static_cast<int>(info.height);
  }

  // 射线投射结果：-2 表示无地图信息（未知/出图），-1 表示已知空闲且未命中
  static constexpr double kRayUnknown = -2.0;
  static constexpr double kRayNoHit = -1.0;

  /**
   * @brief 沿 map 方向做栅格射线投射，返回首次命中已占据栅格的距离
   * @return 命中距离；已知空闲到量程尽头返回 kRayNoHit；
   *         经过未知区域或射出地图返回 kRayUnknown
   */
  double raycast(double origin_x, double origin_y, double angle, double max_range) const
  {
    const auto & info = map_->info;
    const double resolution = info.resolution;
    if (resolution <= 0.0) {
      return kRayUnknown;
    }

    const double cos_a = std::cos(angle);
    const double sin_a = std::sin(angle);
    const double origin_yaw = yawFromQuaternion(info.origin.orientation);
    const double cos_o = std::cos(-origin_yaw);
    const double sin_o = std::sin(-origin_yaw);
    const double step = resolution * 0.5;

    for (double d = step; d <= max_range; d += step) {
      const double wx = origin_x + cos_a * d;
      const double wy = origin_y + sin_a * d;
      const double dx = wx - info.origin.position.x;
      const double dy = wy - info.origin.position.y;
      const int gx = static_cast<int>(std::floor((dx * cos_o - dy * sin_o) / resolution));
      const int gy = static_cast<int>(std::floor((dx * sin_o + dy * cos_o) / resolution));
      if (gx < 0 || gy < 0 ||
        gx >= static_cast<int>(info.width) || gy >= static_cast<int>(info.height))
      {
        return kRayUnknown;
      }
      const int8_t value = map_->data[static_cast<size_t>(gy) * info.width + gx];
      if (value < 0) {
        // 未知栅格：地图不提供该方向的通行/障碍信息
        return kRayUnknown;
      }
      if (value >= occupied_threshold_) {
        return d;
      }
    }
    return kRayNoHit;
  }

  // ── 自动触发 ──
  void evaluate()
  {
    if (secondsSince(start_time_) < startup_grace_period_) {
      return;
    }

    // 稳定期结束后，用静止期的失配中位数作为环境基线
    if (!baseline_ready_ && baseline_samples_.size() >= 20) {
      const size_t mid = baseline_samples_.size() / 2;
      std::nth_element(
        baseline_samples_.begin(), baseline_samples_.begin() + mid, baseline_samples_.end());
      scan_baseline_fraction_ = baseline_samples_[mid];
      baseline_ready_ = true;
      RCLCPP_INFO(
        get_logger(), "激光-地图失配基线 %.2f，实际触发阈值 %.2f",
        scan_baseline_fraction_, effectiveScanThreshold());
    }

    if (!map_ || !have_amcl_ || !scan_received_) {
      return;
    }

    if (recovery_active_) {
      if (secondsSince(recovery_started_) > recovery_timeout_) {
        RCLCPP_ERROR(get_logger(), "初始化定位超时未返回，恢复监控");
        recovery_active_ = false;
        next_trigger_allowed_ = SteadyClock::now() + seconds(recovery_cooldown_);
      }
      return;
    }

    const bool cov_bad = isCovarianceBad();
    const bool scan_bad = isScanMismatchBad();
    if (!cov_bad && !scan_bad) {
      anomaly_active_ = false;
      return;
    }

    // 恢复成功后的稳定期内不重复告警/触发
    if (SteadyClock::now() < settle_until_) {
      anomaly_active_ = false;
      return;
    }

    if (!anomaly_active_) {
      anomaly_active_ = true;
      anomaly_since_ = SteadyClock::now();
      if (anomaly_sustain_time_ > 0.0) {
        RCLCPP_WARN(
          get_logger(), "检测到定位异常：%s%s，持续 %.1fs 后自动重定位",
          cov_bad ? "协方差过大" : "", scan_bad ? "激光与地图不匹配" : "",
          anomaly_sustain_time_);
      } else {
        RCLCPP_WARN(
          get_logger(), "检测到定位异常：%s%s",
          cov_bad ? "协方差过大" : "", scan_bad ? "激光与地图不匹配" : "");
      }
    }

    if (secondsSince(anomaly_since_) < anomaly_sustain_time_) {
      return;
    }
    if (SteadyClock::now() < next_trigger_allowed_ ||
      SteadyClock::now() < settle_until_)
    {
      return;
    }
    if (require_stationary_ && !isStationary()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "定位异常，但小车仍在运动：停车后自动触发初始化定位");
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "定位异常，但 %s 动作服务器不可用", initialize_action_.c_str());
      return;
    }

    triggerRecovery(cov_bad, scan_bad);
  }

  void triggerRecovery(bool cov_bad, bool scan_bad)
  {
    InitializeLocalization::Goal goal;
    rclcpp_action::Client<InitializeLocalization>::SendGoalOptions options;

    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "初始化定位目标被拒绝");
          recovery_active_ = false;
          next_trigger_allowed_ = SteadyClock::now() + seconds(recovery_cooldown_);
        }
      };
    options.feedback_callback =
      [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const InitializeLocalization::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "初始化定位进行中：%s", feedback->state.c_str());
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        onRecoveryResult(result);
      };

    // 重定位会重置粒子分布，静止时 AMCL 不会更新滤波器，
    // 在产生足够位移前不按协方差判定异常
    cov_suppressed_ = have_odom_;
    cov_suppress_ref_x_ = odom_x_;
    cov_suppress_ref_y_ = odom_y_;

    recovery_active_ = true;
    recovery_started_ = SteadyClock::now();
    const std::string reason =
      std::string(cov_bad ? "协方差过大" : "") +
      std::string(scan_bad ? "激光与地图不匹配" : "");
    RCLCPP_WARN(get_logger(), "自动触发初始化定位（原因：%s）", reason.c_str());
    action_client_->async_send_goal(goal, options);
  }

  void onRecoveryResult(const GoalHandle::WrappedResult & result)
  {
    recovery_active_ = false;
    const bool succeeded =
      result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result && result.result->success;
    const auto now = SteadyClock::now();
    next_trigger_allowed_ = now + seconds(recovery_cooldown_);

    if (succeeded) {
      failures_ = 0;
      anomaly_active_ = false;
      settle_until_ = now + seconds(settle_after_success_);
      RCLCPP_INFO(
        get_logger(), "初始化定位成功：%s",
        result.result && !result.result->message.empty() ?
        result.result->message.c_str() : "已收敛");
      return;
    }

    ++failures_;
    if (failures_ >= max_consecutive_recoveries_) {
      failures_ = 0;
      next_trigger_allowed_ = now + seconds(failure_hold_time_);
      RCLCPP_ERROR(
        get_logger(), "初始化定位连续失败 %d 次，暂停 %0.0fs 再尝试",
        max_consecutive_recoveries_, failure_hold_time_);
    } else {
      RCLCPP_WARN(
        get_logger(), "初始化定位失败（第 %d 次），%0.0fs 后重试",
        failures_, recovery_cooldown_);
    }
  }

  // ── 工具函数 ──
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double secondsSince(SteadyClock::time_point time) const
  {
    return std::chrono::duration<double>(SteadyClock::now() - time).count();
  }

  static SteadyClock::duration seconds(double value)
  {
    return std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(value));
  }

  // ── 成员 ──
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<InitializeLocalization>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::array<double, 36> amcl_cov_{};
  double amcl_x_{0.0};
  double amcl_y_{0.0};
  double amcl_yaw_{0.0};
  bool have_amcl_{false};

  bool scan_received_{false};
  double scan_mismatch_fraction_{0.0};
  int scan_valid_beams_{0};
  int scan_mismatch_beams_{0};
  int scan_mismatch_sectors_{0};
  SteadyClock::time_point scan_mismatch_time_{};

  bool baseline_ready_{false};
  double scan_baseline_fraction_{0.0};
  std::vector<double> baseline_samples_;

  bool have_odom_{false};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_vx_{0.0};
  double odom_vy_{0.0};
  double odom_wz_{0.0};
  SteadyClock::time_point odom_time_{};

  bool cov_suppressed_{false};
  double cov_suppress_ref_x_{0.0};
  double cov_suppress_ref_y_{0.0};

  SteadyClock::time_point start_time_{};
  SteadyClock::time_point anomaly_since_{};
  SteadyClock::time_point next_trigger_allowed_{};
  SteadyClock::time_point settle_until_{};
  SteadyClock::time_point recovery_started_{};
  bool anomaly_active_{false};
  bool recovery_active_{false};
  int failures_{0};
};

}  // namespace wheeltec_nav2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wheeltec_nav2::LocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
