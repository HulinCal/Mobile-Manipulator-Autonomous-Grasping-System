// 初始化定位 Navigator：仿 nav2_bt_navigator::NavigateToPoseNavigator 实现，
// 只提供行为树执行所需的几个回调，具体动作全部由 BT XML 描述。
#include <chrono>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_core/behavior_tree_navigator.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wheeltec_nav2/action/initialize_localization.hpp"

namespace wheeltec_nav2
{

class InitializeLocalizationNavigator
  : public nav2_core::BehaviorTreeNavigator<wheeltec_nav2::action::InitializeLocalization>
{
public:
  using ActionT = wheeltec_nav2::action::InitializeLocalization;

  /**
   * @brief action 名称，同时作为 bt_navigator 的 navigators 列表中的 id
   */
  std::string getName() override
  {
    return "initialize_localization";
  }

  /**
   * @brief 默认行为树路径，可通过 bt_navigator 的
   *        default_initialize_localization_bt_xml 参数覆盖
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override
  {
    auto node_shared_ptr = node.lock();
    const std::string param_name = "default_initialize_localization_bt_xml";
    if (!node_shared_ptr->has_parameter(param_name)) {
      node_shared_ptr->declare_parameter(
        param_name,
        ament_index_cpp::get_package_share_directory("wheeltec_nav2") +
        "/param/initialize_localization.xml");
    }

    std::string bt_xml_filename;
    node_shared_ptr->get_parameter(param_name, bt_xml_filename);
    return bt_xml_filename;
  }

protected:
  bool goalReceived(ActionT::Goal::ConstSharedPtr /*goal*/) override
  {
    RCLCPP_INFO(logger_, "收到初始化定位请求：全局重定位 + 旋转 360° + 前进 0.5 米");
    return true;
  }

  void onLoop() override
  {
    // 行为树本身没有进度语义，这里只做低频心跳反馈
    const auto now = clock_->now();
    if (last_feedback_time_.nanoseconds() != 0 &&
      (now - last_feedback_time_).seconds() < 1.0)
    {
      return;
    }
    last_feedback_time_ = now;

    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->state = "initializing";
    feedback->progress = 0.0f;
    bt_action_server_->publishFeedback(feedback);
  }

  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override
  {
    if (goal == nullptr) {
      bt_action_server_->terminatePendingGoal();
      return;
    }
    RCLCPP_INFO(logger_, "初始化定位收到新目标，接受抢占");
    bt_action_server_->acceptPendingGoal();
  }

  void goalCompleted(
    ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status) override
  {
    result->success = final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED;
    result->message = result->success ?
      "初始化定位完成" : "初始化定位失败：行为树未成功结束";

    if (result->success) {
      RCLCPP_INFO(logger_, "%s", result->message.c_str());
    } else {
      RCLCPP_ERROR(logger_, "%s", result->message.c_str());
    }
  }

private:
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace wheeltec_nav2

PLUGINLIB_EXPORT_CLASS(
  wheeltec_nav2::InitializeLocalizationNavigator,
  nav2_core::NavigatorBase)
