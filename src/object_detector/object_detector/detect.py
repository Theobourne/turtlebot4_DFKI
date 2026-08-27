#!/usr/bin/env python3
"""
detect.py — run an off-the-shelf YOLO detector on the OAK-D camera stream.

Detection only: this version does NOT undock. Undock the robot yourself first
(ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"), then run
this. Prints each detection with its confidence and republishes an annotated
image on /detections/annotated/compressed.

    ros2 run object_detector detect
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

# ---- settings you can change ---------------------------------------------
CAMERA_TOPIC = "/oakd/rgb/image_raw/compressed"
OUTPUT_TOPIC = "/detections/annotated/compressed"
DETECT_PER_SECOND = 3
MODEL = "yolov8n.pt"
CONF_THRESHOLD = 0.4
# --------------------------------------------------------------------------


class Detector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.get_logger().info("loading YOLO model...")
        self.model = YOLO(MODEL)
        self.latest = None
        self.warned_waiting = False
        self.announced_frames = False

        self.create_subscription(CompressedImage, CAMERA_TOPIC, self.on_image, 10)
        self.pub = self.create_publisher(CompressedImage, OUTPUT_TOPIC, 10)
        self.create_timer(1.0 / DETECT_PER_SECOND, self.detect)
        self.get_logger().info("YOLO detector ready.")

    def on_image(self, msg):
        self.latest = msg

    def detect(self):
        if self.latest is None:
            if not self.warned_waiting:
                self.get_logger().info("waiting for the first camera frame...")
                self.warned_waiting = True
            return

        if not self.announced_frames:
            self.get_logger().info("receiving frames — detecting.")
            self.announced_frames = True

        buf = np.frombuffer(self.latest.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        results = self.model(frame, conf=CONF_THRESHOLD, verbose=False)[0]

        detections = []
        for box in results.boxes:
            name = self.model.names[int(box.cls)]
            conf = float(box.conf)
            detections.append(f"{name} {conf:.2f}")
        if detections:
            self.get_logger().info("seen: " + ", ".join(detections))

        annotated = results.plot()
        ok, jpeg = cv2.imencode(".jpg", annotated)
        if ok:
            out = CompressedImage()
            out.header = self.latest.header
            out.format = "jpeg"
            out.data = jpeg.tobytes()
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()