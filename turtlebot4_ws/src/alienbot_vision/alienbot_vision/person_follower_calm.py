import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import signal
import time
import math
from ultralytics import YOLO
from std_msgs.msg import Float64

# ── Tuning ────────────────────────────────────────────────────────────────────
CENTER_DEADZONE   = 0.12   # fraction of half-width; increase to reduce twitching
LINEAR_SPEED      = 0.15   # slower than annoy mode (0.20)
ANGULAR_SPEED     = 0.5    # gentler turning than annoy (0.6)
ANGULAR_VEL_GAIN  = 0.15   # less aggressive velocity feed-forward
ANGULAR_DIR       = -1     # -1 for standard non-mirrored camera
SMOOTH_ALPHA      = 0.65   # slightly more smoothing than annoy (0.75)
CMD_TIMEOUT       = 1.0    # seconds — watchdog zeros velocity if no fresh frame

ANKLE_CONF_THRESH = 0.4
FEET_PAD          = 30
MISS_LIMIT        = 20

# ── Follow distance ──────────────────────────────────────────────────────────
# Stop driving when box_area exceeds this — person is close enough
STOP_AREA         = 0.03   # tune: watch area: in logs at desired stop distance
STOP_HYSTERESIS   = 0.7    # resume driving when area drops below STOP_AREA * this
# When stopped, keep turning to face the person but don't drive forward

# Wing flap — only when stopped near person
FLAP_SPEED_IDLE   = 0.3    # gentle flap speed when stopped near person

# Search behavior — spin in place to find a person
SEARCH_SPIN_SPEED = 0.3    # slightly slower search spin than annoy (0.4)
SEARCH_DELAY      = 3.0    # seconds to wait stationary before spinning

# ── Obstacle avoidance (LiDAR) ───────────────────────────────────────────────
TRAVEL_DIR_INDEX  = 180
AVOID_CONE_HALF   = 60
OBSTACLE_STOP_DIST = 0.35
OBSTACLE_SLOW_DIST = 0.80
OBSTACLE_BIAS_DIST = 1.2
AVOID_STEER_GAIN  = 0.5
DEAD_AHEAD_HALF   = 15

LEFT_ANKLE  = 15
RIGHT_ANKLE = 16
# ─────────────────────────────────────────────────────────────────────────────

_IMG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_SCAN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class PersonFollowerCalm(Node):
    def __init__(self):
        super().__init__('person_follower_calm')
        self.bridge  = CvBridge()
        self.model   = YOLO('yolov8n-pose.pt')
        self.pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub = self.create_publisher(Image, '/follower_image', 10)

        # Wing flap speed
        self.flap_speed_pub = self.create_publisher(Float64, '/wing_flap_speed', 10)
        self.current_flap_speed = 0.0

        # Camera subscription
        self.sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, _IMG_QOS)

        # LiDAR subscription
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, _SCAN_QOS)
        self.latest_scan = None
        self.scan_lock = threading.Lock()

        self.locked_feet    = None
        self.locked_area    = None
        self.miss_count     = 0
        self.prev_cx        = None
        self._frame_count   = 0
        self.last_twist     = Twist()
        self.latest_frame   = None
        self.frame_lock     = threading.Lock()
        self.running        = True
        self.last_cmd_time  = time.time()

        # Search state
        self.target_lost_time = None

        # Stop state — true when robot has stopped near person
        self.is_stopped_near = False

        # EMA state
        self.smooth_angular = 0.0
        self.smooth_linear  = 0.0

        self.infer_thread    = threading.Thread(target=self.infer_loop,    daemon=True)
        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.infer_thread.start()
        self.watchdog_thread.start()

        self.get_logger().info(
            'Person follower CALM started — follows from distance, '
            'flaps only when stopped near person'
        )

    # ── Watchdog ──────────────────────────────────────────────────────────
    def watchdog_loop(self):
        while self.running and rclpy.ok():
            time.sleep(0.05)
            if time.time() - self.last_cmd_time > CMD_TIMEOUT:
                try:
                    self.pub.publish(Twist())
                    self.smooth_angular = 0.0
                    self.smooth_linear  = 0.0
                except Exception:
                    pass

    # ── Shutdown ──────────────────────────────────────────────────────────
    def stop_robot(self):
        self.running = False
        try:
            self.flap_speed_pub.publish(Float64(data=0.0))
            for _ in range(5):
                self.pub.publish(Twist())
                time.sleep(0.05)
        except Exception:
            pass

    # ── Camera callback ───────────────────────────────────────────────────
    def image_callback(self, msg):
        buf   = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self.frame_lock:
            self.latest_frame = frame

    # ── LiDAR callback ────────────────────────────────────────────────────
    def scan_callback(self, msg):
        with self.scan_lock:
            self.latest_scan = msg

    # ── Obstacle avoidance ────────────────────────────────────────────────
    def compute_obstacle_avoidance(self):
        with self.scan_lock:
            scan = self.latest_scan

        if scan is None:
            return 1.0, 0.0, 'no_scan'

        ranges = scan.ranges
        n = len(ranges)
        if n == 0:
            return 1.0, 0.0, 'empty_scan'

        def get_range(idx):
            idx = idx % n
            r = ranges[idx]
            if r < scan.range_min or r > scan.range_max or r == 0.0:
                return float('inf')
            return r

        ahead_min = float('inf')
        for i in range(-DEAD_AHEAD_HALF, DEAD_AHEAD_HALF + 1):
            r = get_range(TRAVEL_DIR_INDEX + i)
            ahead_min = min(ahead_min, r)

        left_min = float('inf')
        right_min = float('inf')
        for i in range(DEAD_AHEAD_HALF, AVOID_CONE_HALF + 1):
            r = get_range(TRAVEL_DIR_INDEX + i)
            left_min = min(left_min, r)
            r = get_range(TRAVEL_DIR_INDEX - i)
            right_min = min(right_min, r)

        linear_scale = 1.0
        if ahead_min < OBSTACLE_STOP_DIST:
            linear_scale = 0.0
        elif ahead_min < OBSTACLE_SLOW_DIST:
            linear_scale = (ahead_min - OBSTACLE_STOP_DIST) / (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)

        angular_bias = 0.0
        if left_min < OBSTACLE_BIAS_DIST:
            strength = (OBSTACLE_BIAS_DIST - left_min) / OBSTACLE_BIAS_DIST
            angular_bias -= AVOID_STEER_GAIN * strength
        if right_min < OBSTACLE_BIAS_DIST:
            strength = (OBSTACLE_BIAS_DIST - right_min) / OBSTACLE_BIAS_DIST
            angular_bias += AVOID_STEER_GAIN * strength

        info = f'ahead:{ahead_min:.2f} L:{left_min:.2f} R:{right_min:.2f}'
        return linear_scale, angular_bias, info

    # ── Inference loop ────────────────────────────────────────────────────
    def infer_loop(self):
        while self.running and rclpy.ok():
            with self.frame_lock:
                frame = self.latest_frame
                self.latest_frame = None

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                self.process_frame(frame)
            except Exception as e:
                self.get_logger().error(f'process_frame crashed: {e}')
                self.stop_robot()

        try:
            self.pub.publish(Twist())
        except Exception:
            pass

    # ── Feet extraction ───────────────────────────────────────────────────
    def extract_feet(self, result, w, h):
        feet_list = []
        if result.keypoints is None:
            return feet_list

        for kp in result.keypoints.data:
            kp_np = kp.cpu().numpy()
            left  = kp_np[LEFT_ANKLE]
            right = kp_np[RIGHT_ANKLE]
            l_vis = left[2]  > ANKLE_CONF_THRESH
            r_vis = right[2] > ANKLE_CONF_THRESH

            if not l_vis and not r_vis:
                continue

            pts = []
            if l_vis:
                pts.append((int(left[0]), int(left[1])))
            if r_vis:
                pts.append((int(right[0]), int(right[1])))

            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            x1 = max(0,     min(xs) - FEET_PAD)
            x2 = min(w - 1, max(xs) + FEET_PAD)
            y1 = max(0,     min(ys) - FEET_PAD)
            y2 = min(h - 1, max(ys) + FEET_PAD)
            feet_list.append((cx, cy, x1, y1, x2, y2))

        return feet_list

    # ── Main control ──────────────────────────────────────────────────────
    def process_frame(self, frame):
        if not self.running:
            return

        h, w = frame.shape[:2]
        frame_area = h * w

        results  = self.model(frame, device='cpu', conf=0.2, verbose=False, imgsz=256, half=False)
        all_feet = []
        for result in results:
            all_feet.extend(self.extract_feet(result, w, h))

        twist = Twist()

        # Get obstacle avoidance
        linear_scale, angular_bias, obstacle_info = self.compute_obstacle_avoidance()

        if all_feet:
            self.miss_count = 0

            # Reset search state
            if self.target_lost_time is not None:
                self.get_logger().info('Person found — search stopped!')
                self.target_lost_time = None

            if self.locked_feet is None:
                all_feet.sort(key=lambda f: -f[1])
                self.locked_feet = all_feet[0]
                f = self.locked_feet
                self.locked_area = (f[4] - f[2]) * (f[5] - f[3]) / frame_area
                self.get_logger().info(f'Feet locked! (area: {self.locked_area:.3f})')

            lx, ly = self.locked_feet[0], self.locked_feet[1]
            la = self.locked_area if self.locked_area is not None else 0.0

            # Tight proximity window
            candidates = [f for f in all_feet
                          if abs(f[0] - lx) < w * 0.15 and abs(f[1] - ly) < h * 0.3]

            if la > 0.001:
                candidates = [f for f in candidates
                              if 0.3 < ((f[4]-f[2])*(f[5]-f[3]) / frame_area) / la < 3.0]

            if candidates:
                best = min(candidates, key=lambda f: abs(f[0] - lx) + abs(f[1] - ly))
                self.locked_feet = best

            cx, cy, x1, y1, x2, y2 = self.locked_feet
            box_area   = (x2 - x1) * (y2 - y1) / frame_area

            # Update locked_area with EMA
            if self.locked_area is not None:
                self.locked_area = 0.7 * self.locked_area + 0.3 * box_area
            else:
                self.locked_area = box_area

            h_error = (cx - w / 2) / (w / 2)
            h_vel = (cx - self.prev_cx) / w if self.prev_cx is not None else 0.0
            self.prev_cx = cx

            # ── Turning — always track the person ─────────────────────────
            if abs(h_error) > CENTER_DEADZONE:
                angular_raw = ANGULAR_DIR * (ANGULAR_SPEED * h_error + ANGULAR_VEL_GAIN * h_vel)
                twist.angular.z = angular_raw

            twist.angular.z += angular_bias

            # ── Distance control — stop when close enough ─────────────────
            if box_area > STOP_AREA:
                # Close enough — stop driving, just track
                twist.linear.x = 0.0
                if not self.is_stopped_near:
                    self.is_stopped_near = True
                    self.get_logger().info(f'Close enough (area:{box_area:.3f}) — stopping, flapping gently.')
            elif self.is_stopped_near and box_area < STOP_AREA * STOP_HYSTERESIS:
                # Person moved away — resume following
                self.is_stopped_near = False
                twist.linear.x = -LINEAR_SPEED * linear_scale
                self.get_logger().info(f'Person moved away (area:{box_area:.3f}) — resuming follow.')
            elif not self.is_stopped_near:
                # Normal following
                twist.linear.x = -LINEAR_SPEED * linear_scale
            else:
                # In hysteresis zone — stay stopped
                twist.linear.x = 0.0

            # ── Wing flap — only when stopped near person ─────────────────
            if self.is_stopped_near:
                if self.current_flap_speed != FLAP_SPEED_IDLE:
                    self.flap_speed_pub.publish(Float64(data=FLAP_SPEED_IDLE))
                    self.current_flap_speed = FLAP_SPEED_IDLE
            else:
                if self.current_flap_speed != 0.0:
                    self.flap_speed_pub.publish(Float64(data=0.0))
                    self.current_flap_speed = 0.0

            # EMA smoothing
            self.smooth_angular = SMOOTH_ALPHA * twist.angular.z + (1 - SMOOTH_ALPHA) * self.smooth_angular
            self.smooth_linear  = SMOOTH_ALPHA * twist.linear.x  + (1 - SMOOTH_ALPHA) * self.smooth_linear
            twist.angular.z = self.smooth_angular
            twist.linear.x  = self.smooth_linear

            self.last_twist.linear.x  = twist.linear.x
            self.last_twist.angular.z = twist.angular.z
            self.last_cmd_time = time.time()
            self._frame_count += 1

            # Status tag
            state = 'STOPPED' if self.is_stopped_near else 'FOLLOWING'
            avoid_tag = f' AVOID[{linear_scale:.1f}]' if linear_scale < 1.0 else ''

            self.get_logger().info(
                f'[{self._frame_count}] {state} cx:{cx:.0f} h_error:{h_error:.2f} '
                f'angular:{twist.angular.z:.2f} linear:{twist.linear.x:.2f} '
                f'area:{box_area:.3f} flap:{self.current_flap_speed:.2f} '
                f'det:{len(all_feet)}{avoid_tag}'
            )

            # Visuals
            # Green when following, cyan when stopped
            color = (255, 255, 0) if self.is_stopped_near else (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.circle(frame, (int(cx), int(cy)), 6, color, -1)
            state_text = 'STOPPED - flapping' if self.is_stopped_near else 'FOLLOWING'
            cv2.putText(frame, f'{state_text} area:{box_area:.3f}',
                        (x1, max(y1 - 10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            for f in all_feet:
                if f != self.locked_feet:
                    cv2.rectangle(frame, (int(f[2]), int(f[3])), (int(f[4]), int(f[5])), (0, 0, 255), 1)
                    cv2.putText(frame, 'IGNORED', (int(f[2]), int(f[3]) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            if linear_scale < 1.0:
                bar_color = (0, 0, 255) if linear_scale == 0.0 else (0, 165, 255)
                bar_width = int(w * (1.0 - linear_scale))
                cv2.rectangle(frame, (0, 0), (bar_width, 20), bar_color, -1)
                cv2.putText(frame, f'OBSTACLE {obstacle_info}', (5, 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        else:
            self.miss_count += 1
            if self.miss_count >= MISS_LIMIT:
                self.locked_feet    = None
                self.locked_area    = None
                self.miss_count     = 0
                self.prev_cx        = None
                self.smooth_angular = 0.0
                self.smooth_linear  = 0.0
                self.last_twist     = Twist()
                self.is_stopped_near = False

                if self.current_flap_speed > 0.0:
                    self.flap_speed_pub.publish(Float64(data=0.0))
                    self.current_flap_speed = 0.0

                if self.target_lost_time is None:
                    self.target_lost_time = time.time()
                    self.get_logger().info('Target lost — waiting before search spin...')

                elapsed = time.time() - self.target_lost_time
                if elapsed > SEARCH_DELAY:
                    twist.angular.z = SEARCH_SPIN_SPEED
                    self.get_logger().info('Searching...', throttle_duration_sec=2.0)
            else:
                if self.target_lost_time is not None:
                    elapsed = time.time() - self.target_lost_time
                    if elapsed > SEARCH_DELAY:
                        twist.angular.z = SEARCH_SPIN_SPEED
                else:
                    twist = self.last_twist

        if self.running:
            self.last_cmd_time = time.time()
            self.pub.publish(twist)
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerCalm()

    def shutdown(sig, frame):
        node.get_logger().info(f'Signal {sig} — stopping.')
        node.stop_robot()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGHUP,  shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()