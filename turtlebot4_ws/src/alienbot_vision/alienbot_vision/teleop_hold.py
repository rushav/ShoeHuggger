import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard
import threading

MAX_LINEAR  = 0.26  
MAX_ANGULAR = 1.82

class TeleopHold(Node):
    def __init__(self):
        super().__init__('teleop_hold')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keys = set()
        self.lock = threading.Lock()
        self.timer = self.create_timer(0.05, self.publish_velocity)

    def publish_velocity(self):
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

    def on_press(self, key):
        with self.lock:
            self.keys.add(key)

    def on_release(self, key):
        with self.lock:
            self.keys.discard(key)
        if key == keyboard.Key.esc:
            return False  # stop listener

def main(args=None):
    rclpy.init(args=args)
    node = TeleopHold()
    print("Ready! HOLD arrow keys to move. Release = stop. SPACE = e-stop. Ctrl+C = quit")

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # pynput listener runs in its own thread
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