/**
 * grasp_node.cpp
 * ============================================================================
 * 控制真实机械臂完成抓取
 * ============================================================================
 * 功能：
 *   1. 订阅 detect_markers_node 发布的 /grasp_target_pose (PoseStamped)
 *      —— 该 pose 表示 gripper_base (link6_flange) 在 base_link 下的目标位姿
 *   2. 检测位姿稳定性：滑动窗口 (默认 2.0s) 内 position 偏差 < tolerance
 *      且 quaternion 偏差 < tolerance，则认为位姿已稳定
 *   3. 位姿稳定后，使用 MoveGroupInterface 让 MoveIt 规划到目标位姿
 *   4. 规划成功后执行 trajectory，驱动 ros2_control -> 真实机械臂运动
 *   5. （可选）到达后控制夹爪闭合/张开
 *
 * 节点名：grasp_node
 *
 * 订阅话题：
 *   /grasp_target_pose  (geometry_msgs/PoseStamped)  来自 detect_markers_node
 *
 * 参数：
 *   planning_group       : MoveIt 规划组，默认 "arm"
 *   end_effector_link    : 末端 link，默认 "link6_flange"
 *   pose_topic           : 输入 pose 话题，默认 "/grasp_target_pose"
 *   stable_window        : 稳定性检测窗口 [s]，默认 2.0
 *   stable_pos_tol       : 位置容差 [m]，默认 0.005 (5mm)
 *   stable_rot_tol       : 旋转容差 [rad]，默认 0.02 (~1.1°)
 *   planning_time        : MoveIt 规划超时 [s]，默认 5.0
 *   planning_attempts    : 规划失败后最大重试次数，默认 5
 *   gripper_group        : 夹爪规划组，默认 "gripper"
 *   gripper_close        : 闭合时的 gripper 关节位置，默认 0.0
 *   gripper_open         : 张开时的 gripper 关节位置，默认 0.05
 *   execute_gripper      : 到达目标后是否操作夹爪，默认 true
 *
 * :author hl
 * :date September 2026
 * ============================================================================
 */

#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "tf2/LinearMath/Quaternion.h"

class GraspNode : public rclcpp::Node
{
public:
  GraspNode()
  : rclcpp::Node(
        "grasp_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // ---- 参数 ----
    planning_group_      = this->declare_parameter<std::string>("planning_group", "arm");
    end_effector_link_   = this->declare_parameter<std::string>("end_effector_link", "link6_flange");
    pose_topic_          = this->declare_parameter<std::string>("pose_topic", "/grasp_target_pose");
    stable_window_       = this->declare_parameter<double>("stable_window", 2.0);
    stable_pos_tol_      = this->declare_parameter<double>("stable_pos_tol", 0.04);    // 40mm (放宽，适应小marker位置噪声)
    stable_rot_tol_      = this->declare_parameter<double>("stable_rot_tol", 0.70);     // ~40° (放宽，适应小marker姿态噪声)
    planning_time_       = this->declare_parameter<double>("planning_time", 10.0);
    planning_attempts_   = this->declare_parameter<int>("planning_attempts", 5);
    gripper_group_       = this->declare_parameter<std::string>("gripper_group", "gripper");
    gripper_close_       = this->declare_parameter<double>("gripper_close", 0.0);
    gripper_open_        = this->declare_parameter<double>("gripper_open", 0.05);
    execute_gripper_     = this->declare_parameter<bool>("execute_gripper", true);

    // ---- 订阅 ----
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&GraspNode::poseCallback, this, std::placeholders::_1));

    // ---- 创建独立节点用于 MoveGroupInterface ----
    // 关键：这个节点不能加入主 executor，否则 MoveGroupInterface 内部的
    // rclcpp::spin 会因节点已被占用而失败，导致永远无法就绪。
    // 参考 MoveIt2 官方示例 hello_moveit.cpp 的模式。
    move_group_node_ = std::make_shared<rclcpp::Node>(
        "grasp_move_group_helper",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // ---- 创建 MoveGroupInterface ----
    // 注意：MoveGroupInterface 构造时会阻塞，直到与 move_group 节点建立连接
    // 因此放在单独线程中创建，避免阻塞主线程
    std::thread([this]() {
      RCLCPP_INFO(this->get_logger(),
          "等待 MoveGroupInterface 初始化 (planning_group='%s', end_effector='%s')...",
          planning_group_.c_str(), end_effector_link_.c_str());

      try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            move_group_node_,  // 独立节点，未加入外部 executor
            planning_group_);

        move_group_->setPlanningTime(planning_time_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setMaxVelocityScalingFactor(0.5);       // 安全起见，限速
    move_group_->setMaxAccelerationScalingFactor(0.5);
    // 显式设置工作空间（与 ompl_planning.yaml 中 arm.workspace_bounds 一致）
    // 否则 ValidateWorkspaceBounds 适配器会使用默认值，可能不利于规划
    move_group_->setWorkspace(-0.35, -0.35, 0.0, 0.35, 0.35, 0.5);

        RCLCPP_INFO(this->get_logger(),
            "MoveGroupInterface 就绪. 当前末端位姿: %s",
            formatPose(move_group_->getCurrentPose().pose).c_str());
        RCLCPP_INFO(this->get_logger(),
            "稳定性检测: 窗口=%.2fs pos_tol=%.4fm rot_tol=%.4frad",
            stable_window_, stable_pos_tol_, stable_rot_tol_);
        RCLCPP_INFO(this->get_logger(), "等待接收 %s ...", pose_topic_.c_str());

        move_group_ready_ = true;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(),
            "MoveGroupInterface 初始化失败: %s", e.what());
      }
    }).detach();

    state_ = State::WAITING_FOR_POSE;

    // ---- 诊断定时器：等待 MoveGroupInterface 期间每 3s 打印状态 ----
    diag_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
          if (!move_group_ready_) {
            RCLCPP_WARN(this->get_logger(),
                "仍在等待 MoveGroupInterface 初始化... "
                "(请确认 execute_grasp.launch.py 中 move_group 已启动)");
          } else if (state_ == State::WAITING_FOR_POSE) {
            RCLCPP_INFO(this->get_logger(),
                "MoveGroupInterface 已就绪，等待 %s 发布目标位姿...", pose_topic_.c_str());
          }
        });
  }

  ~GraspNode() override = default;

private:
  // ---- 状态机 ----
  enum class State {
    WAITING_FOR_POSE,   // 等待 pose 输入
    STABILIZING,        // 正在累积窗口，检测稳定性
    PLANNING,           // 位姿稳定，正在规划
    EXECUTING,          // 正在执行 trajectory
    DONE,               // 完成
  };

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_ready_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "MoveGroupInterface 尚未就绪，忽略 pose");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    switch (state_) {
      case State::WAITING_FOR_POSE:
      case State::DONE:
        // 第一次收到 pose，开始累积窗口
        pose_window_.clear();
        pose_window_.push_back(*msg);
        state_ = State::STABILIZING;
        RCLCPP_INFO(this->get_logger(),
            "收到目标位姿，开始稳定性检测 (窗口=%.1fs)...", stable_window_);
        break;

      case State::STABILIZING:
        pose_window_.push_back(*msg);
        // 丢弃窗口外的旧 pose
        while (!pose_window_.empty() &&
               (this->now() - rclcpp::Time(pose_window_.front().header.stamp)).seconds() > stable_window_)
        {
          pose_window_.pop_front();
        }
        // 窗口内有足够多的样本（至少 5 个，避免单点抖动）
        if (pose_window_.size() >= 5) {
          if (checkStable()) {
            state_ = State::PLANNING;
            // 在新线程中规划，避免阻塞订阅
            std::thread([this]() { planAndExecute(); }).detach();
          }
          // checkStable() 内部会打印稳定/不稳定的诊断信息
        }
        break;

      case State::PLANNING:
      case State::EXECUTING:
        // 规划/执行中忽略新的 pose
        break;
    }
  }

  // ---- 检测窗口内位姿是否稳定 ----
  bool checkStable()
  {
    if (pose_window_.size() < 2) return false;

    // 取窗口中所有 pose，计算相对平均值的最大偏差
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto & p : pose_window_) {
      sum_x += p.pose.position.x;
      sum_y += p.pose.position.y;
      sum_z += p.pose.position.z;
    }
    double mean_x = sum_x / pose_window_.size();
    double mean_y = sum_y / pose_window_.size();
    double mean_z = sum_z / pose_window_.size();

    double max_pos_err = 0.0;
    double max_rot_err = 0.0;
    // 直接从消息字段构造，避免 tf2::fromMsg 的链接问题
    const auto & o_ref = pose_window_.front().pose.orientation;
    tf2::Quaternion q_ref(o_ref.x, o_ref.y, o_ref.z, o_ref.w);
    q_ref.normalize();

    for (const auto & p : pose_window_) {
      double dx = p.pose.position.x - mean_x;
      double dy = p.pose.position.y - mean_y;
      double dz = p.pose.position.z - mean_z;
      max_pos_err = std::max(max_pos_err, std::sqrt(dx*dx + dy*dy + dz*dz));

      // 用 quaternion 点积的绝对值衡量旋转偏差（1=完全相同）
      tf2::Quaternion q(p.pose.orientation.x, p.pose.orientation.y,
                        p.pose.orientation.z, p.pose.orientation.w);
      q.normalize();
      double dot = std::abs(q_ref.dot(q));
      if (dot > 1.0) dot = 1.0;
      double angle = 2.0 * std::acos(dot);
      max_rot_err = std::max(max_rot_err, angle);
    }

    bool stable = (max_pos_err < stable_pos_tol_ && max_rot_err < stable_rot_tol_);
    if (stable) {
      RCLCPP_INFO(this->get_logger(),
          "位姿稳定！pos_err=%.4fm rot_err=%.4frad", max_pos_err, max_rot_err);
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "位姿尚未稳定: pos_err=%.4fm (tol=%.4f) rot_err=%.4frad (tol=%.4f) 样本数=%zu",
          max_pos_err, stable_pos_tol_, max_rot_err, stable_rot_tol_, pose_window_.size());
    }
    return stable;
  }

  // ---- 规划 + 执行 ----
  void planAndExecute()
  {
    geometry_msgs::msg::PoseStamped target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 取窗口最后一个 pose 作为目标
      target = pose_window_.back();
    }

    RCLCPP_INFO(this->get_logger(), "=== 开始 MoveIt 规划到目标位姿 ===");
    RCLCPP_INFO(this->get_logger(), "  target: %s", formatPose(target.pose).c_str());

    // ---- 当前末端位姿诊断 ----
    geometry_msgs::msg::PoseStamped current = move_group_->getCurrentPose();
    double dz = target.pose.position.z - current.pose.position.z;
    double dxy = std::sqrt(
        std::pow(target.pose.position.x - current.pose.position.x, 2) +
        std::pow(target.pose.position.y - current.pose.position.y, 2));
    RCLCPP_INFO(this->get_logger(),
        "  当前末端: %s", formatPose(current.pose).c_str());
    RCLCPP_INFO(this->get_logger(),
        "  位移差距: 水平=%.3fm 垂直=%.3fm", dxy, dz);

    // ---- 可达性预检 ----
    // mycobot280 连杆总长约 0.325m，最大水平半径约 0.32m
    // 提前过滤掉物理上不可达的目标，避免无谓的规划尝试
    const double MAX_REACH = 0.32;  // [m] 经验值，略小于连杆总长
    double horiz_dist = std::sqrt(
        target.pose.position.x * target.pose.position.x +
        target.pose.position.y * target.pose.position.y);
    if (horiz_dist > MAX_REACH) {
      RCLCPP_ERROR(this->get_logger(),
          "目标水平距离 %.4fm 超出机械臂可达半径 %.2fm，跳过规划。"
          " 请把 marker 放近一些，或减小 grasp_offset。",
          horiz_dist, MAX_REACH);
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = State::WAITING_FOR_POSE;
      return;
    }
    RCLCPP_INFO(this->get_logger(),
        "可达性预检通过: 水平距离 %.4fm <= %.2fm", horiz_dist, MAX_REACH);

    move_group_->setPoseTarget(target.pose);

    // ---- IK 预校验：检查目标位姿是否能解出 IK ----
    // 如果 IK 都解不出来，OMPL 也无法规划，提前给出诊断
    {
      const moveit::core::JointModelGroup * jmg =
          move_group_->getCurrentState()
              ->getRobotModel()
              ->getJointModelGroup(planning_group_);
      bool ik_success = move_group_->getCurrentState()->setFromIK(
          jmg, target.pose, "", 0.1,
          moveit::core::GroupStateValidityCallbackFn());
      if (!ik_success) {
        RCLCPP_WARN(this->get_logger(),
            "IK 无法求解目标位姿 —— 这通常是姿态/位置不可达。"
            " 检查 marker 的姿态是否符合预期，或调整 grasp_orientation。");
      } else {
        RCLCPP_INFO(this->get_logger(), "IK 预校验通过");
      }
    }

    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (int attempt = 1; attempt <= planning_attempts_; ++attempt) {
      RCLCPP_INFO(this->get_logger(), "规划尝试 %d/%d ...", attempt, planning_attempts_);
      auto result = move_group_->plan(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        success = true;
        RCLCPP_INFO(this->get_logger(), "规划成功 (attempt=%d)", attempt);
        break;
      }
      RCLCPP_WARN(this->get_logger(), "规划失败 (attempt=%d)", attempt);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "规划失败超过最大尝试次数，放弃");
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = State::WAITING_FOR_POSE;
      return;
    }

    // ---- 执行 ----
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = State::EXECUTING;
    }
    RCLCPP_INFO(this->get_logger(), "执行 trajectory ...");
    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "执行失败");
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = State::WAITING_FOR_POSE;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "机械臂到达目标位姿！");

    // ---- 可选：操作夹爪 ----
    if (execute_gripper_) {
      operateGripper(gripper_close_);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      operateGripper(gripper_open_);   // 张开准备放下
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = State::WAITING_FOR_POSE;
    }
    RCLCPP_INFO(this->get_logger(), "抓取流程完成，等待下一次目标 ...");
  }

  // ---- 夹爪控制 ----
  void operateGripper(double value)
  {
    RCLCPP_INFO(this->get_logger(), "夹爪 -> %.3f", value);
    // 用 MoveGroupInterface 控制夹爪 group（复用独立节点）
    moveit::planning_interface::MoveGroupInterface gripper(
        move_group_node_, gripper_group_);
    gripper.setJointValueTarget(gripper_group_, std::vector<double>{value});
    moveit::planning_interface::MoveGroupInterface::Plan gplan;
    if (gripper.plan(gplan) == moveit::core::MoveItErrorCode::SUCCESS) {
      gripper.execute(gplan);
    } else {
      RCLCPP_WARN(this->get_logger(), "夹爪规划失败");
    }
  }

  // ---- 工具：格式化 Pose ----
  static std::string formatPose(const geometry_msgs::msg::Pose & p)
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "pos=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
        p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    return std::string(buf);
  }

  // ---- 成员 ----
  std::string planning_group_, end_effector_link_, pose_topic_, gripper_group_;
  double stable_window_, stable_pos_tol_, stable_rot_tol_, planning_time_;
  int planning_attempts_;
  double gripper_close_, gripper_open_;
  bool execute_gripper_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;  // 诊断定时器
  rclcpp::Node::SharedPtr move_group_node_;  // 独立节点，专供 MoveGroupInterface 使用
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::atomic<bool> move_group_ready_{false};

  State state_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_window_;
  std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GraspNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
