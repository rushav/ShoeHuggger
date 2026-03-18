#!/usr/bin/env python3
"""
AlienBot Teleop — Hold-to-Move
===============================
Manual teleoperation node using arrow keys. Hold a key to move, release to stop.
Unlike standard ROS2 teleop_keyboard, this requires continuous key-hold — releasing
any key immediately stops that axis.

Controls:
    Arrow Up/Down    — forward/backward
    Arrow Left/Right — turn left/right
    Space            — emergency stop (zero all velocities)
    Esc              — quit

Publishes:
    /cmd_vel (geometry_msgs/Twist)
"""

import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

# Maximum velocities (TurtleBot3 Waffle limits)
MAX_LINEAR  = 0.26   # m/s
MAX_ANGULAR = 1.82   # rad/s


class TeleopHold(Node):
    """ROS2 node for hold-to-move teleoperation via arrow keys."""

    def __init__(self) -> None:
        super().__init__('teleop_hold')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keys: set = set()
        self.lock = threading.Lock()
        self.timer = self.create_timer(0.05, self.publish_velocity)

    def publish_velocity(self) -> None:
        """Publish Twist based on currently held keys (called at 20 Hz)."""
        with self.lock:
            twist = Twist()
            if keyboard.Key.space in self.keys:
                self.pub.publish(twist)
                return
            if keyboard.Key.up in self.keys:
                twist.linear.x = MAX_LINEAR
            elif keyboard.Key.down in self.keys:
                twist.linear.x = -MAX_LINEAR
            if keyboard.Key.left in self.keys:
                twist.angular.z = MAX_ANGULAR
            elif keyboard.Key.right in self.keys:
                twist.angular.z = -MAX_ANGULAR
            self.pub.publish(twist)

    def on_press(self, key) -> None:
        """Track key press."""
        with self.lock:
            self.keys.add(key)

    def on_release(self, key) -> bool | None:
        """Track key release. Return False on Esc to stop listener."""
        with self.lock:
            self.keys.discard(key)
        if key == keyboard.Key.esc:
            return False


def main(args=None):
    rclpy.init(args=args)
    node = TeleopHold()
    print('Ready! HOLD arrow keys to move. Release = stop. SPACE = e-stop. Ctrl+C = quit')

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    with keyboard.Listener(
        on_press=node.on_press,
        on_release=node.on_release
    ) as listener:
        try:
            listener.join()
        except KeyboardInterrupt:
            pass

    twist = Twist()
    node.pub.publish(twist)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
