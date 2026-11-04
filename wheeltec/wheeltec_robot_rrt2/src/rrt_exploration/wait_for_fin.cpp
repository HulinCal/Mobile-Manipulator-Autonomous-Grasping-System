/**
 * @file wait_for_fin.cpp
 * @brief 自主建图结束判定与回原点节点
 *
 * 结束判定（双条件，同时满足才触发）：
 *   1. 前沿耗尽：frontier_timeout_sec 内没有收到新的 /detected_frontiers
 *   2. 小车静止：线/角速度均低于阈值且持续 still_duration_sec
 * 且必须至少收到过一次前沿（防止启动初期误判）。
 *
 * 触发后动作序列：
 *   saveMap() 保存地图 -> stopAssigner() 停掉目标分配节点 -> 发布 map 原点
 *   目标位姿(/goal_pose) -> 发布 overmap_flag 完成标志
 */
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cstdlib>
#include <filesystem>
#include <string>

using std::placeholders::_1;

class WaitForFin : public rclcpp::Node
{
public:
    WaitForFin() : Node("wait_for_fin")
    {
        declareParameters();
        readParameters();

        frontier_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/detected_frontiers", 10, std::bind(&WaitForFin::frontierCallback, this, _1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom_combined", 10, std::bind(&WaitForFin::odomCallback, this, _1));

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
        overmap_flag_pub_ = create_publisher<std_msgs::msg::Int8>("overmap_flag", 1);

        tick_timer_ = create_wall_timer(
            std::chrono::seconds(1), std::bind(&WaitForFin::tick, this));

        RCLCPP_INFO(get_logger(),
            "wait_for_fin 启动: 前沿超时=%.0fs, 静止判定时长=%.0fs, 存图=%d, 回原点=%d",
            frontier_timeout_sec_, still_duration_sec_, save_map_, return_to_origin_);
    }

private:
    // ---------- 参数 ----------
    void declareParameters()
    {
        declare_parameter("frontier_timeout_sec", 15.0);
        declare_parameter("still_duration_sec", 5.0);
        declare_parameter("still_linear_eps", 0.005);
        declare_parameter("still_angular_eps", 0.005);
        declare_parameter("save_map", true);
        declare_parameter("map_save_path", std::string(""));
        declare_parameter("return_to_origin", true);
        declare_parameter("goal_publish_repeat", 5);
    }

    void readParameters()
    {
        get_parameter("frontier_timeout_sec", frontier_timeout_sec_);
        get_parameter("still_duration_sec", still_duration_sec_);
        get_parameter("still_linear_eps", still_linear_eps_);
        get_parameter("still_angular_eps", still_angular_eps_);
        get_parameter("save_map", save_map_);
        get_parameter("map_save_path", map_save_path_);
        get_parameter("return_to_origin", return_to_origin_);
        get_parameter("goal_publish_repeat", goal_publish_repeat_);
        if (map_save_path_.empty()) {
            const char* home = std::getenv("HOME");
            map_save_path_ = std::string(home ? home : "/tmp") + "/maps/rrt_map";
        }
    }

    // ---------- 订阅回调（只记录状态，不做判定） ----------
    void frontierCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        if (!msg->markers.empty()) {
            saw_frontiers_ = true;
            last_frontier_time_ = now();
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const bool stationary =
            std::fabs(msg->twist.twist.linear.x) < still_linear_eps_ &&
            std::fabs(msg->twist.twist.angular.z) < still_angular_eps_;
        if (stationary) {
            if (!stationary_since_.has_value()) stationary_since_ = now();
        } else {
            stationary_since_.reset();
        }
    }

    // ---------- 条件判定 ----------
    bool frontierExhausted() const
    {
        return saw_frontiers_ &&
               (now() - last_frontier_time_).seconds() > frontier_timeout_sec_;
    }

    bool stationaryLongEnough() const
    {
        return stationary_since_.has_value() &&
               (now() - stationary_since_.value()).seconds() > still_duration_sec_;
    }

    // ---------- 动作 ----------
    void saveMap()
    {
        if (!save_map_) return;
        const std::filesystem::path dir =
            std::filesystem::path(map_save_path_).parent_path();
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        const std::string cmd =
            "ros2 run nav2_map_server map_saver_cli -t /map -f " + map_save_path_ +
            " --ros-args -p map_subscribe_transient_local:=true";
        RCLCPP_INFO(get_logger(), "保存地图: %s", cmd.c_str());
        const int ret = std::system(cmd.c_str());
        if (ret != 0) RCLCPP_WARN(get_logger(), "地图保存命令返回 %d", ret);
    }

    void stopAssigner()
    {
        // assigner 无 lifecycle 接口，ROS2 下用 pkill 精确停掉，防止其再下发探索目标
        const int ret = std::system("pkill -x assigner");
        RCLCPP_INFO(get_logger(), "停止 assigner (pkill 返回 %d)", ret);
    }

    void publishReturnGoal()
    {
        geometry_msgs::msg::PoseStamped origin;
        origin.header.stamp = now();
        origin.header.frame_id = "map";
        origin.pose.orientation.w = 1.0;   // 位置(0,0) + 朝向yaw=0，其余字段默认为0
        goal_pub_->publish(origin);
    }

    void publishFinishFlag()
    {
        std_msgs::msg::Int8 flag;
        flag.data = 1;
        overmap_flag_pub_->publish(flag);
    }

    // ---------- 主循环（1Hz） ----------
    void tick()
    {
        if (done_) return;

        if (!finished_) {
            if (frontierExhausted() && stationaryLongEnough()) {
                RCLCPP_INFO(get_logger(), "前沿耗尽且小车静止，判定建图完成");
                saveMap();
                stopAssigner();
                finished_ = true;
            }
            return;
        }

        if (return_to_origin_ && goal_published_ < goal_publish_repeat_) {
            publishReturnGoal();   // 分多帧重发，避免与 Nav2 激活时序竞争
            goal_published_++;
            if (goal_published_ == goal_publish_repeat_) {
                publishFinishFlag();
                RCLCPP_INFO(get_logger(), "回原点目标已下发，overmap_flag=1");
                done_ = true;
            }
        } else if (!return_to_origin_) {
            publishFinishFlag();
            done_ = true;
        }
    }

    // ---------- 成员 ----------
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr overmap_flag_pub_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    double frontier_timeout_sec_, still_duration_sec_;
    double still_linear_eps_, still_angular_eps_;
    bool save_map_, return_to_origin_;
    std::string map_save_path_;
    int goal_publish_repeat_;

    bool saw_frontiers_ = false;
    rclcpp::Time last_frontier_time_{0, 0, RCL_ROS_TIME};
    std::optional<rclcpp::Time> stationary_since_;
    bool finished_ = false, done_ = false;
    int goal_published_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaitForFin>());
    rclcpp::shutdown();
    return 0;
}
