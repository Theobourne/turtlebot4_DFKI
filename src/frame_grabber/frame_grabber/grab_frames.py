#!/usr/bin/env python3
"""
grab_frames.py — keep a rolling buffer of recent OAK-D camera frames on disk.

Saves 3 frames/second into a fixed set of 90 slots (30 seconds' worth). The
filenames cycle frame_000.jpg ... frame_089.jpg and then wrap back to 000, so
the folder never grows past 90 files and the numbers never run away. Runs until
stopped with Ctrl+C.

NOTE: because the slot numbers cycle, filename order is NOT time order once the
buffer has filled. To find the newest frame, sort by file modification time
(os.path.getmtime), not by name.

Files go in ~/turtlebot4_ws/oakd_frames/rolling/.

Run once built and sourced, after undocking:
    ros2 run frame_grabber grab_frames
"""

import os
import glob
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# ---- settings you can change ---------------------------------------------
CAMERA_TOPIC = "/oakd/rgb/image_raw/compressed"   # the HD compressed stream
FRAMES_PER_SECOND = 3       # how many images to save each second
WINDOW_SECONDS = 30         # how many seconds of history to keep
# --------------------------------------------------------------------------

MAX_FRAMES = FRAMES_PER_SECOND * WINDOW_SECONDS    # 3 * 30 = 90 slots


class FrameGrabber(Node):
    def __init__(self, out_dir):
        super().__init__("frame_grabber")
        self.out_dir = out_dir
        self.latest = None            # the most recent image that arrived
        self.slot = 0                 # cycles 0..MAX_FRAMES-1, never grows past it
        self.warned_waiting = False   # so the "waiting" note prints only once

        self.create_subscription(CompressedImage, CAMERA_TOPIC, self.on_image, 10)
        self.create_timer(1.0 / FRAMES_PER_SECOND, self.save_frame)

    def on_image(self, msg):
        self.latest = msg             # remember the newest picture

    def save_frame(self):
        if self.latest is None:
            if not self.warned_waiting:
                self.get_logger().warn("waiting for the first frame...")
                self.warned_waiting = True
            return

        # Write into the current slot, overwriting whatever old frame was there.
        filename = os.path.join(self.out_dir, f"frame_{self.slot:03d}.jpg")
        with open(filename, "wb") as f:
            f.write(bytes(self.latest.data))

        # Move to the next slot, wrapping back to 0 after the last one (slot 89).
        self.slot = (self.slot + 1) % MAX_FRAMES


def clear_folder(out_dir):
    """Remove any frames left over from a previous run, so we start fresh."""
    for old in glob.glob(os.path.join(out_dir, "frame_*.jpg")):
        try:
            os.remove(old)
        except OSError:
            pass


def main(args=None):
    out_dir = os.path.expanduser("~/turtlebot4_ws/oakd_frames/rolling")
    os.makedirs(out_dir, exist_ok=True)
    clear_folder(out_dir)

    rclpy.init(args=args)
    node = FrameGrabber(out_dir)
    node.get_logger().info(
        f"Rolling buffer: {FRAMES_PER_SECOND} frames/s into {MAX_FRAMES} slots "
        f"(~{WINDOW_SECONDS}s) in {out_dir}."
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Stopped. Buffer left in {out_dir}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()