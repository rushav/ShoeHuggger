#!/usr/bin/env python3
"""
AlienBot Person Detector — Legacy/Debug
========================================
Simple YOLOv8n person detection node for testing camera connectivity
and basic detection. Draws bounding boxes around detected people.

This is a simpler predecessor to person_follower.py — useful for
verifying that the camera feed and YOLO model work before running
the full follower pipeline.

Subscribes:
    /image_raw (sensor_msgs/Image) — raw camera feed

Publishes:
    /detected_image (sensor_msgs/Image) — annotated image with bounding boxes
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class PersonDetector(Node):
    """ROS2 node that detects people in camera images using YOLOv8n."""

    def __init__(self) -> None:
        super().__init__('person_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/detected_image', 10)
        self.get_logger().info('Person detector started!')

    def image_callback(self, msg: Image) -> None:
        """Run YOLO detection on incoming image and publish annotated result."""
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        results = self.model(
            frame,
            classes=[0],       # person class only
            device='cpu',
            conf=0.15          # low confidence to catch partial views
        )

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'person {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
