#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# AlienBot Launch Script
# Starts robot_state_publisher, SLAM, RViz, and control UI
# ═══════════════════════════════════════════════════════════════════════════════

set -e

source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash
export ROS_DOMAIN_ID=117

echo "═══════════════════════════════════════"
echo "  AlienBot Launch"
echo "═══════════════════════════════════════"

# Check Pi connection
echo "[1/4] Checking robot connection..."
if ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
    echo "  ✓ Robot topics detected"
else
    echo "  ⚠ No robot topics found — is the Pi running?"
fi

# Start robot_state_publisher + SLAM
echo "[2/4] Starting robot_state_publisher + SLAM..."
ros2 launch ~/turtlebot4_ws/alienbot_slam.launch.py &
SLAM_PID=$!
sleep 3

# Start RViz
echo "[3/4] Starting RViz..."
rviz2 -d ~/turtlebot4_ws/alienbot.rviz &
RVIZ_PID=$!
sleep 2

# Start UI
echo "[4/4] Starting AlienBot UI..."
python3 ~/turtlebot4_ws/src/alienbot_vision/alienbot_vision/alienbot_ui.py &
UI_PID=$!

echo ""
echo "═══════════════════════════════════════"
echo "  AlienBot is running!"
echo "═══════════════════════════════════════"
echo "  SLAM+RSP: PID $SLAM_PID"
echo "  RViz:     PID $RVIZ_PID"
echo "  UI:       PID $UI_PID"
echo ""
echo "  Press Ctrl+C to shut everything down"
echo "═══════════════════════════════════════"

cleanup() {
    echo ""
    echo "Shutting down AlienBot..."
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" 2>/dev/null || true
    ros2 topic pub --once /wing_flap_speed std_msgs/msg/Float64 "{data: 0.0}" 2>/dev/null || true
    kill $UI_PID 2>/dev/null || true
    kill $RVIZ_PID 2>/dev/null || true
    kill $SLAM_PID 2>/dev/null || true
    pkill -f "person_follower" 2>/dev/null || true
    pkill -f "person_follower_calm" 2>/dev/null || true
    pkill -f "teleop_hold" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    echo "AlienBot shutdown complete."
    exit 0
}

trap cleanup SIGINT SIGTERM
wait