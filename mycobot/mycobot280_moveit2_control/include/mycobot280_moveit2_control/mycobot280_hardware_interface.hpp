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

#ifndef MYCOBOT280_MOVEIT2_CONTROL__MYCOBOT280_HARDWARE_INTERFACE_HPP_
#define MYCOBOT280_MOVEIT2_CONTROL__MYCOBOT280_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mycobot280_moveit2_control
{

/// ros2_control SystemInterface that bridges MoveIt2 / ros2_control to the Python
/// driver `mycobot_driver_node.py` via std_msgs/Float64MultiArray topics.
///
/// Topic contract:
///   - Publishes 6 arm joint position commands (radians) to `cmd_topic`
///     (default `/mycobot/cmd_joint_pos`).
///   - Subscribes to `state_topic` (default `/mycobot/current_joint_pos`) for
///     the 6 arm joint position states (radians).
///   - Publishes 1 gripper command (radians) to `gripper_cmd_topic`
///     (default `/mycobot/cmd_gripper_pos`).
///   - Subscribes to `gripper_state_topic` (default `/mycobot/current_gripper_pos`)
///     for the current gripper value (radians).
///
/// All joints are in radians. The Python driver converts arm commands to
/// degrees and gripper commands to the native 0..100 scale before calling
/// pymycobot. Gripper velocity is estimated here from successive gripper
/// state messages so the GripperActionController stall detection works.
class MyCobot280HardwareInterface : public hardware_interface::SystemInterface
{
public:
  static constexpr size_t kNoGripper = static_cast<size_t>(-1);

  MyCobot280HardwareInterface();
  ~MyCobot280HardwareInterface() override;

  // Lifecycle hooks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
  override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
  override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Control loop hooks
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// ROS 2 callback that stores the latest arm joint state from the Python driver.
  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  /// ROS 2 callback that stores the latest gripper state from the Python driver.
  void gripper_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Configuration
  std::string cmd_topic_;            // default: /mycobot/cmd_joint_pos
  std::string state_topic_;          // default: /mycobot/current_joint_pos
  std::string gripper_cmd_topic_;    // default: /mycobot/cmd_gripper_pos
  std::string gripper_state_topic_;  // default: /mycobot/current_gripper_pos
  double cmd_publish_rate_ = 50.0;  // Hz at which commands are sent to the driver
  bool dry_run_ = false;            // if true, do not publish to the real arm

  // Number of arm joints (always 6 for mycobot280 arm). Loaded from info_.joints.
  size_t num_joints_ = 0;
  // Index of the gripper_controller joint inside info_.joints. npos if absent
  // (e.g. URDF configured for arm-only without gripper).
  size_t gripper_joint_index_ = kNoGripper;

  // Storage for ros2_control interface values. The StateInterface / CommandInterface
  // objects point into these vectors.
  std::vector<double> hw_states_position_;
  std::vector<double> hw_commands_position_;
  // Gripper velocity state interface required by the GripperActionController.
  // Updated in read() from latest_gripper_velocity_ (estimated in the gripper
  // state callback).
  double hw_states_velocity_gripper_ = 0.0;

  // Latest state from the Python driver, shared between the (non-RT) ROS callback
  // and the (RT) read() method through a realtime-safe box.
  std::vector<double> latest_state_;
  std::mutex latest_state_mutex_;

  // Latest gripper state (single double in a 1-element vector to reuse the
  // Float64MultiArray plumbing) plus velocity estimated from successive
  // state messages (rad/s). Guarded by latest_gripper_state_mutex_.
  std::vector<double> latest_gripper_state_{0.0};
  double latest_gripper_velocity_ = 0.0;
  double prev_gripper_state_for_vel_ = 0.0;
  std::chrono::steady_clock::time_point prev_gripper_state_time_;
  bool prev_gripper_state_received_ = false;
  std::mutex latest_gripper_state_mutex_;
  double pending_gripper_cmd_{0.0};
  std::mutex pending_gripper_cmd_mutex_;

  // ROS 2 publisher / subscriber. Created in on_configure() when the lifecycle
  // node is available.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
  realtime_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
  realtime_gripper_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_state_sub_;

  // Wall-timer driven command publishing: the read/write loop only fills a
  // local buffer; a non-realtime timer actually publishes to keep blocking out
  // of the control loop.
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  std::vector<double> pending_cmd_;
  std::mutex pending_cmd_mutex_;
};

}  // namespace mycobot280_moveit2_control

#endif  // MYCOBOT280_MOVEIT2_CONTROL__MYCOBOT280_HARDWARE_INTERFACE_HPP_
