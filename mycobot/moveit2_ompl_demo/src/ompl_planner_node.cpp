#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// 引入 tf2 用于欧拉角转四元数
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class MoveItPlanner : public rclcpp::Node
{
public:
    MoveItPlanner() : rclcpp::Node("moveit_ompl_planner")
    {
        // 1. 声明欧拉角参数 (单位: 弧度)
        // 如果你希望用户输入角度(degrees)，可以在读取后乘以 M_PI/180.0
        this->declare_parameter<double>("goal_x", 0.2);
        this->declare_parameter<double>("goal_y", 0.0);
        this->declare_parameter<double>("goal_z", 0.2);
        
        // 欧拉角: Roll(X), Pitch(Y), Yaw(Z)
        this->declare_parameter<double>("goal_roll", 0.0);  // 绕X轴
        this->declare_parameter<double>("goal_pitch", 0.0); // 绕Y轴
        this->declare_parameter<double>("goal_yaw", 0.0);   // 绕Z轴

        // 模式开关：position_only=true 时只约束末端 XYZ 位置，姿态由 planner 自由选择。
        // 这对只用 (x,y,z) 指定目标、不关心末端朝向的简单场景很有用，
        // 也避免了 "末端朝上 + z 太低 → 自碰撞" 导致的 GOAL_STATE_INVALID。
        this->declare_parameter<bool>("position_only", true);
        
        RCLCPP_INFO(this->get_logger(), "Node created. Params: goal_x,y,z and goal_roll,pitch,yaw (radians).");
    }

    void initMoveGroup(const std::string& planning_group = "arm")
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group);
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized.");
    }

    bool planAndExecuteFromEulerParams()
    {
        if (!move_group_) {
            RCLCPP_ERROR(this->get_logger(), "MoveGroup not initialized!");
            return false;
        }

        // 1. 设置起始状态
        move_group_->setStartStateToCurrentState();

        // 2. 获取位置参数
        double x = this->get_parameter("goal_x").as_double();
        double y = this->get_parameter("goal_y").as_double();
        double z = this->get_parameter("goal_z").as_double();

        // 3. 获取模式开关
        bool position_only = this->get_parameter("position_only").as_bool();

        if (position_only) {
            // position_only=true：只约束 XYZ 位置，姿态由 planner 自由选择
            move_group_->setPositionTarget(x, y, z);
            RCLCPP_INFO(this->get_logger(), "Position-only target: [%.3f, %.3f, %.3f] (orientation free)",
                        x, y, z);
        } else {
            // 同时约束位置 + 姿态（欧拉角）
            // 注意：当前实现把参数当度数处理（乘了 M_PI/180）。
            // 如果你传弧度，需要在调用时手动换算或者改下面的转换。
            double roll  = this->get_parameter("goal_roll").as_double() * M_PI / 180.0;
            double pitch = this->get_parameter("goal_pitch").as_double() * M_PI / 180.0;
            double yaw   = this->get_parameter("goal_yaw").as_double() * M_PI / 180.0;

            RCLCPP_INFO(this->get_logger(), "Target Pose: Pos[%.3f, %.3f, %.3f], Euler(RPY)[%.3f, %.3f, %.3f]",
                        x, y, z, roll, pitch, yaw);

            // 欧拉角 -> 四元数
            tf2::Quaternion q_tf2;
            q_tf2.setRPY(roll, pitch, yaw);
            q_tf2.normalize();
            geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q_tf2);

            RCLCPP_INFO(this->get_logger(), "Target Pose: Pos[%.3f, %.3f, %.3f], Quaternion[%.3f, %.3f, %.3f, %.3f]",
                        x, y, z, msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w);

            // 构造 PoseStamped
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.frame_id = "base_link";
            goal_msg.header.stamp = this->now();
            goal_msg.pose.position.x = x;
            goal_msg.pose.position.y = y;
            goal_msg.pose.position.z = z;
            goal_msg.pose.orientation = msg_quat;

            move_group_->setPoseTarget(goal_msg);
        }

        RCLCPP_INFO(this->get_logger(), "Planning...");
        auto success = (move_group_->plan(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            return false;
        }

        // 6. 执行
        RCLCPP_INFO(this->get_logger(), "Executing...");
        auto exec_success = (move_group_->execute(current_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!exec_success) {
            RCLCPP_ERROR(this->get_logger(), "Execution failed!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Task completed successfully.");
        return true;
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan current_plan_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MoveItPlanner>();

    // 必须在调用 initMoveGroup() 之前启动一个后台 executor 线程来 spin 节点。
    // 原因：MoveGroupInterface 构造时会创建 action client 并等待 server discovery，
    // 如果没有 executor 在 spin，discovery 回调永远不会被处理 → 卡死。
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 让 executor 跑一会儿，确保节点订阅/discovery 已就绪
    rclcpp::sleep_for(1s);

    node->initMoveGroup();

    // 等待 MoveGroupInterface 内部 action/monitor 完全就绪
    rclcpp::sleep_for(2s);

    // 执行一次
    node->planAndExecuteFromEulerParams();

    // 清理
    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
