#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════════════
# AlienBot Startup Script (Raspberry Pi)
# Called by systemd alienbot.service on boot
# Starts: bringup node, LiDAR driver, USB camera
# ═══════════════════════════════════════════════════════════════════════════════

set -e

# Source ROS2
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=117
export TURTLEBOT3_MODEL=waffle

echo "Starting AlienBot on Raspberry Pi..."

# Start LiDAR driver
echo "[1/3] Starting LiDAR..."
ros2 launch hls_lds_lds01_driver hlds_laser.launch.py &
LIDAR_PID=$!
sleep 2

# Start USB camera with compressed output for WiFi bandwidth
echo "[2/3] Starting camera..."
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -p video_device:="/dev/video0" \
    -p image_width:=640 \
    -p image_height:=480 \
    -p pixel_format:="mjpeg2rgb" \
    -p framerate:=15.0 &
CAMERA_PID=$!
sleep 2

# Start bringup node
echo "[3/3] Starting AlienBot bringup..."
python3 ~/alienbot_bringup.py &
BRINGUP_PID=$!

echo ""
echo "AlienBot Pi started!"
echo "  LiDAR:   PID $LIDAR_PID"
echo "  Camera:  PID $CAMERA_PID"
echo "  Bringup: PID $BRINGUP_PID"

cleanup() {
    echo "Stopping AlienBot Pi..."
    kill $BRINGUP_PID 2>/dev/null || true
    kill $CAMERA_PID 2>/dev/null || true
    kill $LIDAR_PID 2>/dev/null || true
    echo "AlienBot Pi stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM
wait
