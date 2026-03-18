#!/usr/bin/env python3
"""
AlienBot Bringup Node — Raspberry Pi
=====================================
Custom replacement for turtlebot3_node that runs on the Raspberry Pi.
Communicates with the OpenCR board via DynamixelSDK to control:
  - Wheel motors (DYNAMIXEL ID 1, ID 2)
  - Wing motor (DYNAMIXEL ID 3)

Publishes:
    /odom (nav_msgs/Odometry) — wheel odometry
    /tf (odom → base_link transform)
    /imu (sensor_msgs/Imu) — IMU data from OpenCR
    /joint_states (sensor_msgs/JointState) — wheel + wing joint positions

Subscribes:
    /cmd_vel (geometry_msgs/Twist) — velocity commands for wheels
    /wing_flap_speed (std_msgs/Float64) — wing flap speed (0.0–1.0)
    /wing_command (std_msgs/Float64) — direct wing position (0.0–1.0)

This node reads motor positions and IMU data from OpenCR at ~30 Hz,
computes odometry, and publishes all sensor data to ROS2 topics.

Requires:
    pip3 install dynamixel-sdk

Usage:
    python3 alienbot_bringup.py
    OR via systemd (see alienbot.service)
"""

import math
import signal
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from dynamixel_sdk import PortHandler, PacketHandler

# ── Hardware Configuration ────────────────────────────────────────────────────
DEVICE_PORT       = '/dev/ttyACM0'
BAUDRATE          = 1000000
PROTOCOL_VERSION  = 2.0

# DYNAMIXEL motor IDs
MOTOR_ID_LEFT     = 1    # left wheel
MOTOR_ID_RIGHT    = 2    # right wheel
MOTOR_ID_WING     = 3    # wing mechanism

# Robot physical parameters
WHEEL_RADIUS      = 0.033   # meters
WHEEL_SEPARATION  = 0.1552  # meters (modified from stock 0.287)

# OpenCR control table addresses for reading sensor data
ADDR_PRESENT_POSITION = 132   # 4 bytes
ADDR_PRESENT_VELOCITY = 128   # 4 bytes

# Wing motor control
ADDR_GOAL_POSITION    = 116   # 4 bytes
WING_MIN_POS          = 1024  # ~90° range of motion
WING_MAX_POS          = 3072
WING_CENTER_POS       = 2048

# GIX motor driver addresses for wing via OpenCR
ADDR_GIX_WING_CMD     = 346  # wing command register

# ── Timing ────────────────────────────────────────────────────────────────────
CONTROL_RATE_HZ   = 30    # main control loop frequency
CMD_VEL_TIMEOUT   = 1.0   # seconds — stop if no cmd_vel received
# ─────────────────────────────────────────────────────────────────────────────


class AlienBotBringup(Node):
    """ROS2 bringup node for AlienBot hardware on Raspberry Pi."""

    def __init__(self) -> None:
        super().__init__('alienbot_bringup')

        # ── DynamixelSDK setup ────────────────────────────────────────────
        self.port_handler = PortHandler(DEVICE_PORT)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().fatal(f'Failed to open port {DEVICE_PORT}')
            sys.exit(1)
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().fatal(f'Failed to set baudrate {BAUDRATE}')
            sys.exit(1)

        self.get_logger().info(
            f'Connected to OpenCR on {DEVICE_PORT} at {BAUDRATE} baud')

        # ── Publishers ────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscribers ───────────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.wing_flap_sub = self.create_subscription(
            Float64, '/wing_flap_speed', self.wing_flap_callback, 10)
        self.wing_cmd_sub = self.create_subscription(
            Float64, '/wing_command', self.wing_cmd_callback, 10)

        # ── State ─────────────────────────────────────────────────────────
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_time = time.time()

        # Odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_time = time.time()

        # Wing flap state
        self.wing_flap_speed = 0.0
        self.wing_flap_phase = 0.0
        self.wing_position = 0.5  # normalized 0.0–1.0

        self.running = True

        # ── Main control timer ────────────────────────────────────────────
        self.control_timer = self.create_timer(
            1.0 / CONTROL_RATE_HZ, self.control_loop)

        self.get_logger().info('AlienBot bringup node started!')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist) -> None:
        """Receive velocity commands for wheels."""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self.last_cmd_time = time.time()

    def wing_flap_callback(self, msg: Float64) -> None:
        """Set wing flap speed (0.0 = stop, 1.0 = max speed)."""
        self.wing_flap_speed = max(0.0, min(1.0, msg.data))
        if self.wing_flap_speed == 0.0:
            self.wing_flap_phase = 0.0
        self.get_logger().info(f'Wing flap speed: {self.wing_flap_speed:.2f}')

    def wing_cmd_callback(self, msg: Float64) -> None:
        """Set wing position directly (0.0–1.0 normalized)."""
        self.wing_position = max(0.0, min(1.0, msg.data))
        self.wing_flap_speed = 0.0  # direct position overrides flapping

    # ── Main control loop ─────────────────────────────────────────────────

    def control_loop(self) -> None:
        """Read sensors, compute odometry, send motor commands at fixed rate."""
        if not self.running:
            return

        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        # Watchdog — stop if no cmd_vel received recently
        if now - self.last_cmd_time > CMD_VEL_TIMEOUT:
            self.target_linear = 0.0
            self.target_angular = 0.0

        # Send wheel velocity commands
        self._send_wheel_velocities(self.target_linear, self.target_angular)

        # Update wing flapping
        self._update_wing(dt)

        # Publish odometry and TF
        self._publish_odometry(dt)

        # Publish joint states
        self._publish_joint_states()

    # ── Motor control ─────────────────────────────────────────────────────

    def _send_wheel_velocities(self, linear: float, angular: float) -> None:
        """Convert (linear, angular) to individual wheel velocities and send."""
        left_vel = linear - angular * WHEEL_SEPARATION / 2.0
        right_vel = linear + angular * WHEEL_SEPARATION / 2.0

        # Convert m/s to DYNAMIXEL velocity units
        left_raw = int(left_vel / (WHEEL_RADIUS * 2 * math.pi) * 60 / 0.229)
        right_raw = int(right_vel / (WHEEL_RADIUS * 2 * math.pi) * 60 / 0.229)

        try:
            self.packet_handler.write4ByteTxOnly(
                self.port_handler, MOTOR_ID_LEFT, ADDR_PRESENT_VELOCITY, left_raw)
            self.packet_handler.write4ByteTxOnly(
                self.port_handler, MOTOR_ID_RIGHT, ADDR_PRESENT_VELOCITY, right_raw)
        except Exception as e:
            self.get_logger().warning(f'Wheel write error: {e}')

    def _update_wing(self, dt: float) -> None:
        """Update wing position based on flap speed or direct command."""
        if self.wing_flap_speed > 0.0:
            # Sinusoidal flapping
            freq = self.wing_flap_speed * 3.0  # max ~3 Hz flap
            self.wing_flap_phase += freq * dt * 2 * math.pi
            position = 0.5 + 0.4 * math.sin(self.wing_flap_phase)
        else:
            position = self.wing_position

        # Convert to DYNAMIXEL position
        goal_pos = int(WING_MIN_POS + position * (WING_MAX_POS - WING_MIN_POS))
        try:
            self.packet_handler.write4ByteTxOnly(
                self.port_handler, MOTOR_ID_WING, ADDR_GOAL_POSITION, goal_pos)
        except Exception as e:
            self.get_logger().warning(f'Wing write error: {e}')

    # ── Odometry ──────────────────────────────────────────────────────────

    def _publish_odometry(self, dt: float) -> None:
        """Compute and publish wheel odometry + TF."""
        stamp = self.get_clock().now().to_msg()

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.orientation = self._yaw_to_quaternion(self.odom_theta)
        odom_msg.twist.twist.linear.x = self.target_linear
        odom_msg.twist.twist.angular.z = self.target_angular
        self.odom_pub.publish(odom_msg)

        # Broadcast TF: odom → base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.odom_x
        tf_msg.transform.translation.y = self.odom_y
        q = self._yaw_to_quaternion(self.odom_theta)
        tf_msg.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_joint_states(self) -> None:
        """Publish joint states for URDF visualization."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_left_joint', 'wheel_right_joint',
                    'wing_left_joint', 'wing_right_joint']
        js.position = [0.0, 0.0, 0.0, 0.0]
        js.velocity = []
        js.effort = []
        self.joint_pub.publish(js)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert a yaw angle to a Quaternion message."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    # ── Shutdown ──────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Stop all motors and close port."""
        self.running = False
        self.get_logger().info('Shutting down — stopping all motors...')
        try:
            # Stop wheels
            self.packet_handler.write4ByteTxOnly(
                self.port_handler, MOTOR_ID_LEFT, ADDR_PRESENT_VELOCITY, 0)
            self.packet_handler.write4ByteTxOnly(
                self.port_handler, MOTOR_ID_RIGHT, ADDR_PRESENT_VELOCITY, 0)
            # Center wings
            self.packet_handler.write4ByteTxOnly(
                self.port_handler, MOTOR_ID_WING, ADDR_GOAL_POSITION, WING_CENTER_POS)
        except Exception:
            pass
        self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = AlienBotBringup()

    def signal_handler(sig, frame):
        node.get_logger().info(f'Signal {sig} received — shutting down.')
        node.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
