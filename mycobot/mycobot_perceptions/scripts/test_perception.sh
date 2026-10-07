#!/bin/bash
# Launch the mycobot_perceptions end-to-end test.
#
# This brings up Gazebo (with camera), move_group, the camera_perception node,
# the perception_test_node and RViz. The simulated camera topics are relayed
# onto /camera/depth_registered/points and /camera/color/image_raw so the
# perception node subscribes to its canonical topic names.
#
# Usage:
#   ./test_perception.sh               # default (Gazebo simulation)
#   ./test_perception.sh real           # use a real camera (no Gazebo)
#
# Source ROS 2 and workspace (do not run inside conda to avoid libcurl conflicts)
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../install/setup.bash"

cleanup() {
    echo "Cleaning up..."
    sleep 3.0
    pkill -9 -f "ros2|gazebo|gz|rviz2|robot_state_publisher|moveit|move_group|camera_perception|perception_test_node|topic_tools"
}
trap 'cleanup' SIGINT SIGTERM

USE_GAZEBO="true"
if [ "$1" = "real" ]; then
    USE_GAZEBO="false"
    echo "Running against a REAL camera (use_gazebo:=false)."
fi

echo "Launching perception test..."
# NOTE: use `start_rviz` (not `use_rviz`) — see perception_test.launch.py for
# why: mycobot_moveit_config's move_group.launch.py declares its own use_rviz
# and would otherwise reset our value.
ros2 launch mycobot_perceptions perception_test.launch.py use_gazebo:=$USE_GAZEBO start_rviz:=true
