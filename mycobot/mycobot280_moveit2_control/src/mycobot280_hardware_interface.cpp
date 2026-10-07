// Copyright 2025 hl
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the hl nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "mycobot280_moveit2_control/mycobot280_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mycobot280_moveit2_control
{

namespace
{
constexpr char kPluginName[] = "MyCobot280HardwareInterface";
constexpr char kDefaultCmdTopic[] = "/mycobot/cmd_joint_pos";
constexpr char kDefaultStateTopic[] = "/mycobot/current_joint_pos";
constexpr char kDefaultGripperCmdTopic[] = "/mycobot/cmd_gripper_pos";
constexpr char kDefaultGripperStateTopic[] = "/mycobot/current_gripper_pos";
constexpr char kGripperJointName[] = "gripper_controller";
}  // namespace

MyCobot280HardwareInterface::MyCobot280HardwareInterface() = default;

MyCobot280HardwareInterface::~MyCobot280HardwareInterface() = default;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyCobot280HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Defer to the base class to populate info_ and the lifecycle node plumbing.
  if (auto ret = SystemInterface::on_init(params);
    ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Sanity-check the URDF joint list. The mycobot280 arm exposes 6 actuated
  // joints; the URDF may additionally declare the `gripper_controller` joint
  // for the gripper. We detect the gripper joint by name and treat its value
  // as the native 0..100 pymycobot open ratio (NOT radians).
  const auto & joints = info_.joints;
  gripper_joint_index_ = kNoGripper;
  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].name == kGripperJointName) {
      gripper_joint_index_ = i;
      break;
    }
  }

  // Total joints in the URDF block (arm + optional gripper).
  const size_t total_joints = joints.size();
  const size_t arm_joints = (gripper_joint_index_ == kNoGripper)
    ? total_joints
    : total_joints - 1;
  if (arm_joints != 6) {
    RCLCPP_ERROR(
      get_logger(),
      "[%s] Expected exactly 6 arm joints in the ros2_control block, got %zu. The Python "
      "driver (mycobot_driver_node.py) only handles the 6 arm joints (link1_to_link2 .. "
      "link6_to_link6_flange).",
      kPluginName, arm_joints);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Optionally read parameters from the URDF <param> entries.
  const auto get_param = [&](const std::string & key,
    const std::string & default_value) -> std::string {
      auto it = info_.hardware_parameters.find(key);
      return it == info_.hardware_parameters.end() ? default_value : it->second;
    };
  cmd_topic_ = get_param("cmd_topic", kDefaultCmdTopic);
  state_topic_ = get_param("state_topic", kDefaultStateTopic);
  gripper_cmd_topic_ = get_param("gripper_cmd_topic", kDefaultGripperCmdTopic);
  gripper_state_topic_ = get_param("gripper_state_topic", kDefaultGripperStateTopic);

  const auto dry_run_str = get_param("dry_run", "false");
  dry_run_ = (dry_run_str == "true" || dry_run_str == "1" || dry_run_str == "True");

  // 默认 50 Hz：与 ros2_control update_rate=100Hz 匹配，每个 update 周期
  // 不直接发命令到 driver（driver 是 Python 串口，太频繁会卡串口），
  // 由 cmd_timer_ 以 50Hz（每 20ms）发出。20Hz 会丢一半轨迹点 → 抖动。
  const auto rate_str = get_param("cmd_publish_rate", "50.0");
  try {
    cmd_publish_rate_ = std::stod(rate_str);
  } catch (...) {
    RCLCPP_WARN(
      get_logger(), "[%s] Could not parse cmd_publish_rate='%s', using 50.0 Hz",
      kPluginName, rate_str.c_str());
    cmd_publish_rate_ = 50.0;
  }
  if (cmd_publish_rate_ <= 0.0) {
    cmd_publish_rate_ = 50.0;
  }

  num_joints_ = total_joints;
  hw_states_position_.assign(num_joints_, 0.0);
  hw_commands_position_.assign(num_joints_, 0.0);
  latest_state_.assign(arm_joints, 0.0);
  pending_cmd_.assign(arm_joints, 0.0);

  RCLCPP_INFO(
    get_logger(),
    "[%s] Initialized. Joints: %zu (arm=%zu, gripper=%s) | arm cmd: %s state: %s | "
    "gripper cmd: %s state: %s | rate: %.1f Hz | dry_run: %s",
    kPluginName, num_joints_, arm_joints,
    gripper_joint_index_ == kNoGripper ? "absent" : "present",
    cmd_topic_.c_str(), state_topic_.c_str(),
    gripper_cmd_topic_.c_str(), gripper_state_topic_.c_str(),
    cmd_publish_rate_, dry_run_ ? "true" : "false");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
MyCobot280HardwareInterface::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  interfaces.reserve(num_joints_ + (gripper_joint_index_ != kNoGripper ? 1 : 0));
  for (size_t i = 0; i < num_joints_; ++i) {
    interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  // The GripperActionController additionally requires a velocity state
  // interface for the gripper joint. We don't have a real velocity estimate
  // (the Python driver only reports position), so we expose a static zero
  // placeholder.
  if (gripper_joint_index_ != kNoGripper) {
    interfaces.emplace_back(std::make_shared<hardware_interface::StateInterface>(
      info_.joints[gripper_joint_index_].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_states_velocity_gripper_));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
MyCobot280HardwareInterface::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.reserve(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    interfaces.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
  }
  return interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyCobot280HardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(get_logger(), "[%s] Lifecycle node is not available in on_configure", kPluginName);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // Reset to defaults so an UNCONFIGURED->INACTIVE->UNCONFIGURED cycle works.
  std::fill(hw_states_position_.begin(), hw_states_position_.end(), 0.0);
  std::fill(hw_commands_position_.begin(), hw_commands_position_.end(), 0.0);
  std::fill(latest_state_.begin(), latest_state_.end(), 0.0);
  std::fill(pending_cmd_.begin(), pending_cmd_.end(), 0.0);
  {
    std::lock_guard<std::mutex> g(latest_gripper_state_mutex_);
    latest_gripper_state_.assign(1, 0.0);
    latest_gripper_velocity_ = 0.0;
    prev_gripper_state_received_ = false;
  }
  {
    std::lock_guard<std::mutex> g(pending_gripper_cmd_mutex_);
    pending_gripper_cmd_ = 0.0;
  }

  // Publisher for arm joint commands going to the Python driver.
  cmd_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, 5);
  realtime_cmd_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(cmd_pub_);

  // Subscription for arm joint states coming back from the Python driver.
  state_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    state_topic_, 5,
    std::bind(&MyCobot280HardwareInterface::state_callback, this, std::placeholders::_1));

  if (gripper_joint_index_ != kNoGripper) {
    gripper_cmd_pub_ =
      node->create_publisher<std_msgs::msg::Float64MultiArray>(gripper_cmd_topic_, 5);
    realtime_gripper_cmd_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
        gripper_cmd_pub_);
    gripper_state_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      gripper_state_topic_, 5,
      std::bind(&MyCobot280HardwareInterface::gripper_state_callback, this,
        std::placeholders::_1));
  }

  // Timer-driven publishing of the pending command. We do not publish inside
  // write() to keep the control loop deterministic; this non-RT timer thread
  // takes care of the (potentially blocking) publish call.
  const auto period_ns = std::chrono::nanoseconds(
    static_cast<int64_t>(1e9 / cmd_publish_rate_));
  cmd_timer_ = node->create_wall_timer(
    period_ns,
    [this]() {
      if (dry_run_) {
        return;
      }
      // Arm joints.
      std::vector<double> cmd;
      {
        std::lock_guard<std::mutex> guard(pending_cmd_mutex_);
        cmd = pending_cmd_;
      }
      if (realtime_cmd_pub_ && realtime_cmd_pub_->trylock()) {
        realtime_cmd_pub_->msg_.data.assign(cmd.begin(), cmd.end());
        realtime_cmd_pub_->unlockAndPublish();
      }
      // Gripper (1-element array of the 0..100 open ratio).
      if (realtime_gripper_cmd_pub_) {
        double gripper_cmd = 0.0;
        {
          std::lock_guard<std::mutex> g(pending_gripper_cmd_mutex_);
          gripper_cmd = pending_gripper_cmd_;
        }
        if (realtime_gripper_cmd_pub_->trylock()) {
          realtime_gripper_cmd_pub_->msg_.data.assign(1, gripper_cmd);
          realtime_gripper_cmd_pub_->unlockAndPublish();
        }
      }
    });

  RCLCPP_INFO(
    get_logger(), "[%s] Configured. Topics: arm cmd=%s state=%s | gripper cmd=%s state=%s",
    kPluginName,
    cmd_topic_.c_str(), state_topic_.c_str(),
    gripper_cmd_topic_.c_str(), gripper_state_topic_.c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyCobot280HardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  cmd_timer_.reset();
  gripper_state_sub_.reset();
  realtime_gripper_cmd_pub_.reset();
  gripper_cmd_pub_.reset();
  state_sub_.reset();
  realtime_cmd_pub_.reset();
  cmd_pub_.reset();
  RCLCPP_INFO(get_logger(), "[%s] Cleaned up", kPluginName);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyCobot280HardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Copy the current state into the command buffer so the arm does not jump on
  // activation. If we have never received state, this is just zeros (which
  // corresponds to the arm's home position in radians). The gripper command is
  // similarly seeded with the latest reported gripper value.
  {
    std::lock_guard<std::mutex> guard(pending_cmd_mutex_);
    pending_cmd_ = latest_state_;
    // Copy arm joints into hw_commands_position_ at the corresponding indices
    // (gripper slot is preserved untouched below).
    for (size_t i = 0, arm_i = 0; i < num_joints_; ++i) {
      if (i == gripper_joint_index_) {
        continue;
      }
      if (arm_i < latest_state_.size()) {
        hw_commands_position_[i] = latest_state_[arm_i];
      }
      ++arm_i;
    }
  }
  if (gripper_joint_index_ != kNoGripper) {
    double gripper_state = 0.0;
    {
      std::lock_guard<std::mutex> g(latest_gripper_state_mutex_);
      if (!latest_gripper_state_.empty()) {
        gripper_state = latest_gripper_state_[0];
      }
    }
    hw_commands_position_[gripper_joint_index_] = gripper_state;
    std::lock_guard<std::mutex> g(pending_gripper_cmd_mutex_);
    pending_gripper_cmd_ = gripper_state;
  }
  RCLCPP_INFO(get_logger(), "[%s] Activated. Commands seeded with current state.", kPluginName);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MyCobot280HardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivated", kPluginName);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MyCobot280HardwareInterface::state_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // The arm state from the Python driver is a 6-element array of radians.
  if (!msg || msg->data.size() != latest_state_.size()) {
    return;
  }
  std::lock_guard<std::mutex> guard(latest_state_mutex_);
  latest_state_.assign(msg->data.begin(), msg->data.end());
}

void MyCobot280HardwareInterface::gripper_state_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // The gripper state from the Python driver is a 1-element array in radians.
  if (!msg || msg->data.empty()) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> guard(latest_gripper_state_mutex_);
  // Estimate velocity from successive messages. The driver publishes at
  // ~20 Hz even while the value is unchanged (-> 0 velocity), so the action
  // controller's stall detection sees a true stop when the jaws contact.
  if (prev_gripper_state_received_) {
    const double dt =
      std::chrono::duration<double>(now - prev_gripper_state_time_).count();
    if (dt > 1e-6) {
      latest_gripper_velocity_ =
        (msg->data[0] - prev_gripper_state_for_vel_) / dt;
    }
  }
  prev_gripper_state_for_vel_ = msg->data[0];
  prev_gripper_state_time_ = now;
  prev_gripper_state_received_ = true;
  latest_gripper_state_.assign(1, msg->data[0]);
}

hardware_interface::return_type MyCobot280HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Copy the most recent arm state from the driver callback into the ros2_control
  // state interface storage. This keeps the read path allocation-free.
  {
    std::lock_guard<std::mutex> guard(latest_state_mutex_);
    for (size_t i = 0, arm_i = 0; i < num_joints_; ++i) {
      if (i == gripper_joint_index_) {
        continue;
      }
      if (arm_i < latest_state_.size()) {
        hw_states_position_[i] = latest_state_[arm_i];
      }
      ++arm_i;
    }
  }
  // Gripper state (radians) and estimated velocity go into its state slots.
  if (gripper_joint_index_ != kNoGripper) {
    std::lock_guard<std::mutex> guard(latest_gripper_state_mutex_);
    if (!latest_gripper_state_.empty()) {
      hw_states_position_[gripper_joint_index_] = latest_gripper_state_[0];
    }
    hw_states_velocity_gripper_ = latest_gripper_velocity_;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobot280HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Hand off the latest arm command values to the non-RT timer thread which
  // will actually publish them to the Python driver.
  {
    std::lock_guard<std::mutex> guard(pending_cmd_mutex_);
    for (size_t i = 0, arm_i = 0; i < num_joints_; ++i) {
      if (i == gripper_joint_index_) {
        continue;
      }
      if (arm_i < pending_cmd_.size()) {
        pending_cmd_[arm_i] = hw_commands_position_[i];
      }
      ++arm_i;
    }
  }
  // Gripper command (0..100).
  if (gripper_joint_index_ != kNoGripper) {
    std::lock_guard<std::mutex> g(pending_gripper_cmd_mutex_);
    pending_gripper_cmd_ = hw_commands_position_[gripper_joint_index_];
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mycobot280_moveit2_control

// Export the plugin so pluginlib can discover it.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mycobot280_moveit2_control::MyCobot280HardwareInterface,
  hardware_interface::SystemInterface)
