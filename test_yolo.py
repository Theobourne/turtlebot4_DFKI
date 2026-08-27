#!/usr/bin/env python3
"""
test_yolo.py — quick check that YOLO works, OUTSIDE of ROS.

Runs a pretrained YOLOv8 detector on an image and:
  - prints each detected object and its confidence
  - saves an annotated copy (with boxes) as detection_result.jpg

Usage:
    python3 test_yolo.py                 # uses the newest frame in the rolling folder
    python3 test_yolo.py <image.jpg>     # or a specific image you name
"""

import sys
import glob
import os
from ultralytics import YOLO

FRAMES_DIR = os.path.expanduser("~/turtlebot4_ws/oakd_frames/rolling")


def newest_frame():
    """Return the path of the most recent frame in the rolling folder."""
    files = glob.glob(os.path.join(FRAMES_DIR, "*.jpg"))
    if not files:
        return None
    # Files are named in time order, so the max name is the newest. (Using the
    # file's modification time also works: key=os.path.getmtime.)
    return max(files, key=os.path.getmtime)


def main():
    # Use a path given on the command line, otherwise grab the latest frame.
    if len(sys.argv) >= 2:
        image_path = sys.argv[1]
    else:
        image_path = newest_frame()
        if image_path is None:
            print(f"No .jpg frames found in {FRAMES_DIR}")
            print("Run the frame grabber first, or pass an image path.")
            return
        print(f"Using latest frame: {image_path}")

    model = YOLO("yolov8n.pt")     # first run downloads the weights (~6 MB)
    results = model(image_path)

    for r in results:
        r.save(filename="detection_result.jpg")
        if len(r.boxes) == 0:
            print("No objects detected.")
        for box in r.boxes:
            name = model.names[int(box.cls)]
            conf = float(box.conf)
            print(f"{name}: {conf:.2f}")

    print("\nAnnotated image saved as detection_result.jpg")


if __name__ == "__main__":
    main()