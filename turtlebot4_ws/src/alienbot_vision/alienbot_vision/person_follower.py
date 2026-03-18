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
LINEAR_SPEED      = 0.20
ANGULAR_SPEED     = 0.6
ANGULAR_VEL_GAIN  = 0.2    # extra turn boost proportional to feet movement speed
ANGULAR_DIR       = -1     # -1 for standard non-mirrored camera; +1 if image is mirrored
SMOOTH_ALPHA      = 0.75   # EMA smoothing: 0=no response, 1=no smoothing
CMD_TIMEOUT       = 1.0    # seconds — watchdog zeros velocity if no fresh frame

ANKLE_CONF_THRESH = 0.4
FEET_PAD          = 30
MISS_LIMIT        = 20     # at ~4fps inference → ~5s to drop lock; scale up if throughput improves

# Wing flap speed mapping — tune by watching area: in logs
FLAP_AREA_MIN     = 0.01   # minimum area to start flapping (person barely visible)
FLAP_AREA_MAX     = 0.10   # area at which flapping is at max speed (person very close)
FLAP_SPEED_MIN    = 0.15   # slowest flap when person just detected (far away)
FLAP_SPEED_MAX    = 1.0    # fastest flap when person is very close

# Search behavior — spin in place to find a person
SEARCH_SPIN_SPEED = 0.4    # rad/s — slow spin to search for people
SEARCH_DELAY      = 3.0    # seconds to wait stationary before spinning

# ── Obstacle avoidance (LiDAR) ───────────────────────────────────────────────
# Robot drives backward (linear.x < 0), so direction of travel ≈ index 180 (180°)
# LDS-01: 360 readings, 1° per step, index 0 = front of robot
TRAVEL_DIR_INDEX  = 180    # center index of the direction of travel
AVOID_CONE_HALF   = 60     # check ±60° around travel direction (120° total cone)
OBSTACLE_STOP_DIST = 0.35  # meters — stop all forward motion if anything this close dead ahead
OBSTACLE_SLOW_DIST = 0.80  # meters — start slowing down
OBSTACLE_BIAS_DIST = 1.2  # meters — start biasing steering away from obstacle
AVOID_STEER_GAIN  = 0.5    # how aggressively to steer away (rad/s per obstacle)
DEAD_AHEAD_HALF   = 15     # ±15° cone for "dead ahead" stop check

LEFT_ANKLE  = 15
RIGHT_ANKLE = 16
# ─────────────────────────────────────────────────────────────────────────────

# BEST_EFFORT + depth=1: always use the freshest frame, drop stale ones over WiFi
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


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.bridge  = CvBridge()  # used only for /follower_image output
        self.model   = YOLO('yolov8n-pose.pt')
        self.pub     = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub = self.create_publisher(Image, '/follower_image', 10)

        # Wing flap speed: 0.0 = stop, 0.15 = slow, 1.0 = max
        self.flap_speed_pub = self.create_publisher(Float64, '/wing_flap_speed', 10)
        self.current_flap_speed = 0.0

        # Subscribe to compressed topic — ~15x less WiFi bandwidth than raw Image
        self.sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, _IMG_QOS)

        # Subscribe to LiDAR for obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, _SCAN_QOS)
        self.latest_scan = None
        self.scan_lock = threading.Lock()

        self.locked_feet    = None
        self.locked_area    = None   # track target size for filtering impostors
        self.miss_count     = 0
        self.prev_cx        = None   # for velocity-based angular component
        self._frame_count   = 0
        self.last_twist     = Twist()  # replayed on brief detection gaps to prevent micro-stops
        self.latest_frame   = None
        self.frame_lock     = threading.Lock()
        self.running        = True
        self.last_cmd_time  = time.time()

        # Search state
        self.target_lost_time = None

        # EMA state
        self.smooth_angular = 0.0
        self.smooth_linear  = 0.0

        self.infer_thread    = threading.Thread(target=self.infer_loop,    daemon=True)
        self.watchdog_thread = threading.Thread(target=self.watchdog_loop, daemon=True)
        self.infer_thread.start()
        self.watchdog_thread.start()

        self.get_logger().info(
            'Person follower started — subscribed to /image_raw/compressed + /scan, '
            'device=cpu, imgsz=256'
        )

    # ── Watchdog ──────────────────────────────────────────────────────────────
    def watchdog_loop(self):
        """Zero velocity if inference hasn't produced a command recently."""
        while self.running and rclpy.ok():
            time.sleep(0.05)
            if time.time() - self.last_cmd_time > CMD_TIMEOUT:
                try:
                    self.pub.publish(Twist())
                    self.smooth_angular = 0.0
                    self.smooth_linear  = 0.0
                except Exception:
                    pass

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def stop_robot(self):
        self.running = False
        try:
            self.flap_speed_pub.publish(Float64(data=0.0))
            for _ in range(5):
                self.pub.publish(Twist())
                time.sleep(0.05)
        except Exception:
            pass

    # ── Camera callback ───────────────────────────────────────────────────────
    def image_callback(self, msg):
        buf   = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self.frame_lock:
            self.latest_frame = frame

    # ── LiDAR callback ────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        with self.scan_lock:
            self.latest_scan = msg

    # ── Obstacle avoidance ────────────────────────────────────────────────────
    def compute_obstacle_avoidance(self):
        """
        Analyze LiDAR scan and return (linear_scale, angular_bias, obstacle_info).
        - linear_scale: 0.0 (full stop) to 1.0 (no obstacle)
        - angular_bias: positive = turn left, negative = turn right
        - obstacle_info: string for logging
        """
        with self.scan_lock:
            scan = self.latest_scan

        if scan is None:
            return 1.0, 0.0, 'no_scan'

        ranges = scan.ranges
        n = len(ranges)
        if n == 0:
            return 1.0, 0.0, 'empty_scan'

        # Helper: get valid range at index (wraps around, ignores 0.0 = invalid)
        def get_range(idx):
            idx = idx % n
            r = ranges[idx]
            if r < scan.range_min or r > scan.range_max or r == 0.0:
                return float('inf')
            return r

        # ── Dead ahead check (narrow cone) ────────────────────────────────
        ahead_min = float('inf')
        for i in range(-DEAD_AHEAD_HALF, DEAD_AHEAD_HALF + 1):
            r = get_range(TRAVEL_DIR_INDEX + i)
            ahead_min = min(ahead_min, r)

        # ── Left and right sector check ───────────────────────────────────
        # Left sector: travel_dir + 15° to travel_dir + 60° (obstacle on robot's left)
        # Right sector: travel_dir - 60° to travel_dir - 15° (obstacle on robot's right)
        left_min = float('inf')
        right_min = float('inf')

        for i in range(DEAD_AHEAD_HALF, AVOID_CONE_HALF + 1):
            # Left side of travel direction
            r = get_range(TRAVEL_DIR_INDEX + i)
            left_min = min(left_min, r)
            # Right side of travel direction
            r = get_range(TRAVEL_DIR_INDEX - i)
            right_min = min(right_min, r)

        # ── Compute linear scale ──────────────────────────────────────────
        linear_scale = 1.0
        if ahead_min < OBSTACLE_STOP_DIST:
            linear_scale = 0.0
        elif ahead_min < OBSTACLE_SLOW_DIST:
            # Linearly scale from 0 to 1 between stop and slow distances
            linear_scale = (ahead_min - OBSTACLE_STOP_DIST) / (OBSTACLE_SLOW_DIST - OBSTACLE_STOP_DIST)

        # ── Compute angular bias ──────────────────────────────────────────
        angular_bias = 0.0
        if left_min < OBSTACLE_BIAS_DIST:
            # Obstacle on left → steer right (negative angular)
            strength = (OBSTACLE_BIAS_DIST - left_min) / OBSTACLE_BIAS_DIST
            angular_bias -= AVOID_STEER_GAIN * strength

        if right_min < OBSTACLE_BIAS_DIST:
            # Obstacle on right → steer left (positive angular)
            strength = (OBSTACLE_BIAS_DIST - right_min) / OBSTACLE_BIAS_DIST
            angular_bias += AVOID_STEER_GAIN * strength

        # Build info string
        info = f'ahead:{ahead_min:.2f} L:{left_min:.2f} R:{right_min:.2f}'

        return linear_scale, angular_bias, info

    # ── Inference loop ────────────────────────────────────────────────────────
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

    # ── Feet extraction ───────────────────────────────────────────────────────
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

    # ── Flap speed from area ──────────────────────────────────────────────────
    def compute_flap_speed(self, box_area):
        """Map box_area to flap speed: larger area = closer person = faster flap."""
        if box_area < FLAP_AREA_MIN:
            return 0.0

        t = (box_area - FLAP_AREA_MIN) / (FLAP_AREA_MAX - FLAP_AREA_MIN)
        t = max(0.0, min(1.0, t))
        speed = FLAP_SPEED_MIN + t * (FLAP_SPEED_MAX - FLAP_SPEED_MIN)
        return speed

    # ── Main control ──────────────────────────────────────────────────────────
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

        # ── Get obstacle avoidance output ─────────────────────────────────
        linear_scale, angular_bias, obstacle_info = self.compute_obstacle_avoidance()

        if all_feet:
            self.miss_count = 0

            # Reset search state when person found
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

            # Tight proximity window — only match feet near last known position
            candidates = [f for f in all_feet
                          if abs(f[0] - lx) < w * 0.15 and abs(f[1] - ly) < h * 0.3]

            # Filter by size consistency — reject feet >3x or <0.3x the locked size
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

            # h_error: positive = person right of center, negative = left
            h_error    = (cx - w / 2) / (w / 2)

            # Velocity component
            h_vel = (cx - self.prev_cx) / w if self.prev_cx is not None else 0.0
            self.prev_cx = cx

            # Turning — camera-based steering + obstacle avoidance bias
            if abs(h_error) > CENTER_DEADZONE:
                angular_raw = ANGULAR_DIR * (ANGULAR_SPEED * h_error + ANGULAR_VEL_GAIN * h_vel)
                twist.angular.z = angular_raw

            # Add obstacle avoidance angular bias
            twist.angular.z += angular_bias

            # Linear — drive toward feet, scaled by obstacle proximity
            twist.linear.x = -LINEAR_SPEED * linear_scale

            # ── Wing flap speed — always flap when person seen ────────────
            flap_speed = self.compute_flap_speed(box_area)
            if abs(flap_speed - self.current_flap_speed) > 0.05:
                self.flap_speed_pub.publish(Float64(data=flap_speed))
                self.current_flap_speed = flap_speed

            # EMA smoothing
            self.smooth_angular = SMOOTH_ALPHA * twist.angular.z + (1 - SMOOTH_ALPHA) * self.smooth_angular
            self.smooth_linear  = SMOOTH_ALPHA * twist.linear.x  + (1 - SMOOTH_ALPHA) * self.smooth_linear
            twist.angular.z = self.smooth_angular
            twist.linear.x  = self.smooth_linear

            self.last_twist.linear.x  = twist.linear.x
            self.last_twist.angular.z = twist.angular.z
            self.last_cmd_time = time.time()
            self._frame_count += 1

            # Log with obstacle info
            avoid_tag = ''
            if linear_scale < 1.0:
                avoid_tag = f' AVOID[scale:{linear_scale:.1f} bias:{angular_bias:.2f}]'

            self.get_logger().info(
                f'[{self._frame_count}] cx:{cx:.0f} h_error:{h_error:.2f} '
                f'angular:{twist.angular.z:.2f} linear:{twist.linear.x:.2f} '
                f'area:{box_area:.3f} lock_area:{self.locked_area:.3f} '
                f'flap_spd:{flap_speed:.2f} det:{len(all_feet)}'
                f'{avoid_tag}'
            )

            # Visuals
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame,    (int(cx), int(cy)), 6, (0, 255, 0), -1)
            spd_text = f'flap:{flap_speed:.0%}' if flap_speed > 0 else ''
            avoid_text = f' AVOID' if linear_scale < 1.0 else ''
            cv2.putText(frame, f'area:{box_area:.3f} herr:{h_error:.2f} {spd_text}{avoid_text}',
                        (x1, max(y1 - 10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Draw rejected feet in red
            for f in all_feet:
                if f != self.locked_feet:
                    cv2.rectangle(frame, (int(f[2]), int(f[3])), (int(f[4]), int(f[5])), (0, 0, 255), 1)
                    cv2.putText(frame, 'IGNORED', (int(f[2]), int(f[3]) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # Draw obstacle status bar at top of frame
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
                # Stop flapping when target lost
                if self.current_flap_speed > 0.0:
                    self.flap_speed_pub.publish(Float64(data=0.0))
                    self.current_flap_speed = 0.0

                # Start or continue search behavior
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
    node = PersonFollower()

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