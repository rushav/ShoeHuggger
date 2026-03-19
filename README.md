# AlienBot

![AlienBot](media/robot_photo.jpg)
<!-- PLACEHOLDER: Replace with actual photo of AlienBot -->

## About

AlienBot is a personal butler/pet robot built on a modified TurtleBot3 Waffle platform for TECHIN 516 at UW GIX (Winter 2025). It uses YOLOv8 pose estimation to detect and follow people by tracking ankle keypoints, features motorized wings that flap based on proximity, and avoids obstacles using LiDAR reactive avoidance.

### Demo Video
<!-- PLACEHOLDER: Add link to demo video -->
[Watch the demo video]([https://youtube.com/your-video-link](https://youtu.be/eQ_k0fX2IQM])

## Features

- **Person Following**: Detects feet using YOLOv8n-pose ankle keypoints and follows the locked target
- **Two Modes**:
  - **Annoy Mode**: Aggressively chases person, wings flap faster as it gets closer
  - **Follow Mode**: Calmly follows from a distance, wings flap gently when stopped near person
- **Search Behavior**: Spins in place to find a new target when person is lost
- **Target Locking**: Locks onto one person's feet and ignores others using proximity + size filtering
- **Obstacle Avoidance**: Reactive LiDAR-based avoidance — slows/stops/steers around obstacles
- **Wing Mechanism**: 3rd DYNAMIXEL motor drives symmetric wings with variable-speed flapping
- **Control UI**: Tkinter GUI with mode switching, teleop, and emergency stop
- **SLAM Visualization**: Real-time map building in RViz alongside robot model
- **Auto-Start**: Pi bringup runs on boot via systemd — just power on and go

## Hardware

- TurtleBot3 Waffle (OpenCR + Raspberry Pi 4)
- 3rd DYNAMIXEL XL430 motor (ID 3) for wing mechanism
- TRAUSI 1080p USB webcam (rear-facing)
- LDS-01 LiDAR
- Custom 3D-printed chassis and wings (designed in SolidWorks)

## Team Contributions

| Member | Contributions |
|--------|--------------|
| **Rushav** | Camera integration, person follower code (YOLO pose estimation, control logic, obstacle avoidance), custom bringup node, Pi systemd setup, UI, system integration |
| **Eason** | Robot 3D model design (SolidWorks), URDF creation and export, RViz visualization setup |
| **Phoenix** | Subscriber/publisher node architecture, ROS2 communication design |

## Architecture

```
+---------------------+     WiFi (ROS2 DDS)     +----------------------+
|   Raspberry Pi 4    |<------------------------>|   Laptop (Docker)    |
|                     |     ROS_DOMAIN_ID=117    |                      |
| alienbot_bringup.py |                          | person_follower.py   |
|  +- /cmd_vel (sub)  |                          |  +- /cmd_vel (pub)   |
|  +- /joint_states   |                          |  +- /wing_flap_speed |
|  +- /tf (odom->base)|                          |  +- /follower_image  |
|  +- /imu            |                          |                      |
|  +- /odom           |                          | alienbot_ui.py       |
|  +- /wing_flap_speed|                          | teleop_hold.py       |
|                     |                          | rviz2 + slam_toolbox |
| usb_cam_node        |                          +----------------------+
|  +- /image_raw/*    |
|                     |
| hlds_laser_publisher|
|  +- /scan           |
+---------------------+
```

## Setup Instructions

### Prerequisites
- Docker and VS Code with Dev Containers extension
- ROS2 Humble (installed in container)
- TurtleBot3 Waffle with OpenCR board flashed with GIX firmware

### 1. Clone the repository
```bash
git clone https://github.com/YOUR_USERNAME/AlienBot.git
cd AlienBot
```

### 2. Open in Dev Container
Open the folder in VS Code and select "Reopen in Container". The Dockerfile installs all dependencies automatically.

### 3. Build the workspace
```bash
cd ~/turtlebot4_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Set up the Raspberry Pi
Copy the Pi files and enable auto-start:
```bash
scp pi/alienbot_bringup.py ubuntu@<PI_IP>:~/alienbot_bringup.py
scp pi/start_alienbot.sh ubuntu@<PI_IP>:~/start_alienbot.sh
scp pi/alienbot.service ubuntu@<PI_IP>:~/alienbot.service
ssh ubuntu@<PI_IP>
chmod +x ~/start_alienbot.sh
sudo cp ~/alienbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable alienbot
sudo systemctl start alienbot
```

Pi dependencies:
```bash
pip3 install dynamixel-sdk --break-system-packages
sudo apt install ros-humble-compressed-image-transport
```

### 5. Install OpenCR Firmware
Flash the modified GIX firmware using Arduino IDE. See `firmware/README.md` for details.

## Usage

### One-Command Launch (Laptop)
```bash
./launch/launch_alienbot.sh
```
This opens the control UI and RViz with SLAM. Use the UI buttons or keyboard shortcuts:

| Button | Key | Mode |
|--------|-----|------|
| TELEOP | 1 | Manual arrow-key control |
| ANNOY | 2 | Aggressive person following + flapping |
| FOLLOW | 3 | Calm follow from distance |
| STOP ALL | 4 / Space / Esc | Emergency stop all motors |

### Manual Commands
```bash
# Test wheels
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10

# Test wing position
ros2 topic pub --once /wing_command std_msgs/msg/Float64 "{data: 0.5}"

# Test wing flapping
ros2 topic pub --once /wing_flap_speed std_msgs/msg/Float64 "{data: 0.5}"

# Stop flapping
ros2 topic pub --once /wing_flap_speed std_msgs/msg/Float64 "{data: 0.0}"
```

## License

This project is licensed under the Apache License 2.0 — see [LICENSE](LICENSE) for details.
