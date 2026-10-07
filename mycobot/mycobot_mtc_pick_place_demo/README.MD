## 一、mycobot_mtc_pick_place_demo（带感知的抓取放置演示）
功能 ：基于 MoveIt Task Constructor (MTC)，结合 RGBD 相机点云，自动识别物体并完成抓取-放置任务。包含：

- get_planning_scene_server ：从点云提取物体/支撑面，更新 MoveIt planning scene
- mtc_node ：MTC 任务执行器（抓取→移动→放置→回零）
- point_cloud_viewer ：可视化点云处理中间结果（聚类、平面分割等）
### 使用流程
方式 1：一键脚本（推荐）
cd /home/hl/mycobot/mycobot_ros2-main
bash mycobot_mtc_pick_place_demo/scripts/robot.sh
脚本会依次启动：Gazebo（含相机）→ MoveIt move_group → 相机视角调整 → get_planning_scene_server → pick_place_demo。 关闭按 Ctrl+C 。

方式 2：分步手动启动 （便于调试）
# 终端1：Gazebo + 相机 + MoveIt
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py \
    world_file:=pick_and_place_demo.world use_camera:=true use_rviz:=false

# 终端2：MoveIt move_group
source install/setup.bash
ros2 launch mycobot_moveit_config move_group.launch.py \
    rviz_config_file:=mtc_demos.rviz rviz_config_package:=mycobot_mtc_demos

# 终端3：planning scene 服务（感知）
source install/setup.bash
ros2 launch mycobot_mtc_pick_place_demo get_planning_scene_server.launch.py

# 终端4：MTC 抓取放置节点
source install/setup.bash
ros2 launch mycobot_mtc_pick_place_demo pick_place_demo.launch.py

可选：查看点云处理中间结果
# 终端5：点云可视化
source install/setup.bash
ros2 launch mycobot_mtc_pick_place_demo point_cloud_viewer.launch.py \
    file_name:=/tmp/5_objects_cloud_debug_cloud.pcd
# 或查看其他中间结果：/tmp/4_convertToPCL_debug_cloud.pcd、/tmp/5_support_plane_debug_cloud.pcd
脚本 pointcloud.sh 等价于上述 Gazebo+MoveIt+scene_server+viewer 的组合。
### 可调参数
- config/mtc_node_params.yaml ：MTC 节点参数（抓取姿态生成、物体尺寸等）
- config/get_planning_scene_server.yaml ：感知参数（聚类容差、平面分割阈值）