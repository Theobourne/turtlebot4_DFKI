#!/usr/bin/env python3
"""
detect_3d.py — YOLO detection with map-frame localization.

Extends detect.py: as well as saying *what* it sees, this node says *where the
thing is* on the map. For each detection it reads the depth image at the box
centre, turns that pixel into a 3-D point in the camera frame, and transforms
it into the map frame with tf2.

    ros2 run object_detector detect_3d
    ros2 run object_detector detect_3d --ros-args -p target_class:=chair

Published:
    /detections/annotated/compressed  annotated image (view in rqt_image_view)
    /detections/object_point          PointStamped, one per accepted detection
    /detections/markers               MarkerArray for RViz

--- why the compressed topics ---------------------------------------------
This node deliberately uses /oakd/rgb/image_raw/compressed and
/oakd/stereo/image_raw/compressedDepth, NOT the raw topics.

The raw topics are 2.7 MB and 1.8 MB per frame. Over the lab Wi-Fi they get
fragmented into UDP packets and essentially never arrive intact — measured
0.0 Hz at the laptop while the very same topics ran at 4 and 10 Hz on the Pi.
The compressed pair gets through at ~2 Hz of matched pairs, which is plenty
for a stop-and-look search.

compressedDepth is PNG, which is *lossless*, so the decoded image is the
identical uint16 1280x720 millimetre image the raw topic would have carried.
Same resolution means the RGB intrinsics apply to it unchanged and a box pixel
still indexes straight into it.
"""

import collections
import time

import numpy as np
import cv2
import rclpy
import message_filters
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PointStamped with tf2

from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from ultralytics import YOLO

# ---- settings you can change ---------------------------------------------
RGB_TOPIC = "/oakd/rgb/image_raw/compressed"
DEPTH_TOPIC = "/oakd/stereo/image_raw/compressedDepth"
INFO_TOPIC = "/oakd/rgb/camera_info"

ANNOTATED_TOPIC = "/detections/annotated/compressed"
POINT_TOPIC = "/detections/object_point"
MARKER_TOPIC = "/detections/markers"

# Measured on this laptop (i7-8565U, CPU only — there is no usable GPU) against
# live OAK-D frames at 1280x720, conf 0.4.
#
# Two scenes, 30 live frames each. EASY = chair centred, clear, a few metres
# off. HARD = chair close and cropped by the frame edge, low angle, which is
# the case that actually fails in the room.
#
#                     EASY chair@0.4        HARD chair@0.4   junk classes (hard)
#   yolov8n   49 ms   30/30  min 0.42        0/30            traffic light x30,
#                                                            airplane x18
#   yolo11n   56 ms   30/30  min 0.91        0/30            none
#   yolov8s  120 ms   30/30  min 0.92        0/30            traffic light x27
#   yolo11s  120 ms   30/30  min 0.83       30/30  0.78-0.90 none
#   yolo11m  294 ms   30/30  min 0.86       30/30  0.69-0.85 none
#
# The easy scene is nearly useless for choosing: everything scores 30/30 and the
# ranking there is noise. The hard scene decides it. Every small model returns
# ZERO detections — not weak ones, none at all, even with the threshold dropped
# to 0.25 — while yolo11s finds the chair in every frame. yolo11n's excellent
# 0.91 on the easy chair bought nothing where it mattered.
#
# yolov8n additionally reports a traffic light in all 30 hard frames. Harmless
# while hunting a chair, since anything not target_class is dropped below, but
# worth knowing before trusting it on a class you have not tested.
#
# Cost: the camera sustains ~28 Hz, so yolo11s at 120 ms throttles sightings to
# ~8 Hz. That is still ~25 frames inside one DWELL_SECONDS window against
# MIN_SIGHTINGS=3, and comfortably above CUE_MIN_SIGHTINGS=2 while driving, so
# the pipeline does not notice. yolo11m costs 2.5x more for lower confidence
# than yolo11s on both scenes — there is no reason to reach for it.
#
# The heartbeat prints inference time beside the pair rate, so a model that
# genuinely starves the search is visible at a glance.
#
# Raising imgsz above 640 is a trap: these weights are trained at 640, and at
# 960 the same chair fell from 0.83 to 0.51 for twice the compute.
MODEL = "yolov8n.pt"
CONF_THRESHOLD = 0.4

# Cap how much of the CPU inference may take.
#
# Nav2's controller runs on this same laptop and has to publish cmd_vel at about
# 20 Hz. Torch defaults to 4 threads; on a 4-core machine one inference then
# occupies every core, the controller misses its deadline, and velocity commands
# reach the robot in bursts — which is felt as stuttering, lurching motion. It
# looks like a network problem and is not one.
#
# Leaving cores free costs some inference speed and buys smooth driving, which
# is the better trade: a search spends far more time driving than looking.
TORCH_THREADS = 2
TARGET_FRAME = "map"

# The OAK-D's stereo baseline is small, so depth degrades with range. Beyond
# ~6 m the numbers are still *produced* but are not trustworthy enough to put
# on a map, and closer than ~0.3 m is inside the stereo blind spot.
MIN_DEPTH_M = 0.3
MAX_DEPTH_M = 6.0

# A single pixel drops out on dark, shiny or textureless surfaces (the depth
# image comes back 0 there), so sample a patch and take the median of the
# pixels that actually returned. If the small patch is mostly holes, widen once.
PATCH = 5

# Fallback intrinsics, used only until the first camera_info arrives.
FALLBACK_K = dict(fx=1027.002, fy=1026.493, cx=635.739, cy=373.907)
# --------------------------------------------------------------------------

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"

try:
    # Ultralytics' own per-class palette, so our label matches the box it belongs to.
    from ultralytics.utils.plotting import colors as _yolo_colors
except ImportError:  # pragma: no cover - only if ultralytics moves this
    _yolo_colors = None


def draw_range_label(img, xyxy, text, cls_id):
    """Draw the range at a box's top-right corner, styled like YOLO's own label.

    Ultralytics writes "class conf" at the top-LEFT, so the range goes to the
    right to keep the two from colliding, and reuses that class's colour so the
    two labels read as one annotation rather than two competing ones.
    """
    x1, y1, x2, y2 = (int(v) for v in xyxy)
    h, w = img.shape[:2]

    # Same line-width / font-scale rule Ultralytics uses, so the two labels come
    # out the same size whatever the image resolution.
    lw = max(round((h + w) / 2 * 0.003), 2)
    sf = lw / 3.0
    tf = max(lw - 1, 1)
    pad = max(lw, 3)

    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, sf, tf)
    box_w, box_h = tw + 2 * pad, th + 2 * pad

    # Sit just inside the top-right corner: inside keeps the label on-image even
    # when the box is jammed against an edge, which is exactly when a label
    # placed above the box would be clipped away.
    right = min(max(x2, box_w), w)
    top = min(max(y1, 0), max(h - box_h, 0))

    color = tuple(int(c) for c in _yolo_colors(cls_id, True)) if _yolo_colors else (0, 140, 255)
    cv2.rectangle(img, (right - box_w, top), (right, top + box_h), color, -1, cv2.LINE_AA)
    cv2.putText(
        img, text, (right - box_w + pad, top + pad + th),
        cv2.FONT_HERSHEY_SIMPLEX, sf, (255, 255, 255), tf, cv2.LINE_AA,
    )


def decode_compressed_depth(msg):
    """Decode a compressedDepth message into a uint16 image of millimetres.

    The payload is a small binary header followed by a PNG. The header is 12
    bytes in practice, but rather than trust that we just look for the PNG
    signature, which is unambiguous.
    """
    buf = bytes(msg.data)
    start = buf.find(PNG_MAGIC)
    if start < 0:
        return None
    img = cv2.imdecode(np.frombuffer(buf[start:], np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None or img.dtype != np.uint16:
        return None
    return img


class Detector3D(Node):
    def __init__(self):
        super().__init__("object_detector_3d")

        self.declare_parameter("target_class", "")
        self.declare_parameter("model", MODEL)
        self.declare_parameter("conf_threshold", CONF_THRESHOLD)
        self.declare_parameter("target_frame", TARGET_FRAME)
        self.declare_parameter("min_depth", MIN_DEPTH_M)
        self.declare_parameter("max_depth", MAX_DEPTH_M)
        self.declare_parameter("patch", PATCH)

        self.target_class = self.get_parameter("target_class").value.strip().lower()
        self.conf = float(self.get_parameter("conf_threshold").value)
        self.target_frame = self.get_parameter("target_frame").value
        # What was asked for, vs. what we can currently deliver (see
        # check_target_frame): only 'map' has a fallback.
        self.desired_frame = self.target_frame
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.patch = int(self.get_parameter("patch").value)

        self.declare_parameter("torch_threads", TORCH_THREADS)
        threads = int(self.get_parameter("torch_threads").value)
        if threads > 0:
            import torch
            torch.set_num_threads(threads)
            self.get_logger().info(
                f"limiting inference to {threads} threads so Nav2's controller "
                f"keeps its CPU (set torch_threads:=0 to disable)")

        model_name = self.get_parameter("model").value
        self.get_logger().info(f"loading YOLO model {model_name}...")
        t0 = time.time()
        self.model = YOLO(model_name)
        self.get_logger().info(f"loaded in {time.time() - t0:.1f}s")
        # Rolling inference time, reported in the heartbeat. A heavier model is
        # only worth it if the pipeline can still feed the search: the sweep
        # needs MIN_SIGHTINGS frames inside DWELL_SECONDS, and spotting things
        # while driving needs more than that.
        self.infer_ms = None

        self.K = None
        self.announced = False
        self.tf_warned = False
        self.marker_id = 0

        # Tallies for the heartbeat, reset each time it reports.
        self.counts = dict.fromkeys(
            ("rgb", "depth", "pairs", "yolo", "wrong_class",
             "no_depth", "out_of_range", "tf_fail", "published"), 0
        )
        # What was actually recognised, by name. A bare "10 not 'chair'" says
        # nothing when a search comes back empty; "dining table x4, bottle x3"
        # tells you the camera was working and pointed somewhere sensible.
        self.seen = collections.Counter()
        self.heartbeat_period = 5.0
        # "never arrived" and "arrived, then stopped" are different faults with
        # different fixes, so the heartbeat has to remember which it is seeing.
        self.ever_saw_camera = False
        self.ever_saw_depth = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, 10)

        # Use RELIABLE (the default depth-10 profile), NOT qos_profile_sensor_data.
        # The camera publishes these topics as RELIABLE, and over Wi-Fi that is
        # what makes them usable: a 177 KB JPEG is split across many UDP
        # fragments, and a BEST_EFFORT subscriber throws away every sample that
        # loses even one of them — measured 0 pairs in 40 s. RELIABLE asks for
        # the missing fragments again and yields ~2 Hz of matched pairs.
        rgb_sub = message_filters.Subscriber(self, CompressedImage, RGB_TOPIC, qos_profile=10)
        depth_sub = message_filters.Subscriber(self, CompressedImage, DEPTH_TOPIC, qos_profile=10)
        # RGB and depth come out of the RGBD pipeline sharing a timestamp
        # exactly, but RGB arrives ~3x more often, so the synchroniser is what
        # picks out the frames that actually have a depth partner.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=30, slop=0.15
        )
        self.sync.registerCallback(self.on_pair)

        # Counting the two streams separately is what makes a silent node
        # diagnosable: "no RGB", "no depth" and "both arriving but never
        # pairing up" are three different faults with three different fixes,
        # and from the synchroniser alone they all look like nothing happening.
        rgb_sub.registerCallback(lambda _m: self.bump("rgb"))
        depth_sub.registerCallback(lambda _m: self.mark_depth())

        self.pub_img = self.create_publisher(CompressedImage, ANNOTATED_TOPIC, 10)
        self.pub_point = self.create_publisher(PointStamped, POINT_TOPIC, 10)
        self.pub_marker = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)

        what = self.target_class if self.target_class else "any object"
        self.get_logger().info(f"ready — looking for {what}, reporting in '{self.target_frame}'")

        # 'map' only exists once Nav2/AMCL is up. Rather than sit there warning
        # about a missing transform, drop back to the robot's own frame so the
        # node is useful on its own — every log line names the frame it used,
        # so there is no doubt about which one you are looking at.
        self.create_timer(5.0, self.check_target_frame)
        self.create_timer(self.heartbeat_period, self.heartbeat)

    def bump(self, key, n=1):
        self.counts[key] += n

    def mark_depth(self):
        self.counts["depth"] += 1
        self.ever_saw_depth = True

    def heartbeat(self):
        """Say what happened in the last few seconds, even when that is nothing.

        Without this the node is completely silent whenever it sees no objects,
        which is indistinguishable from a dead camera or a broken synchroniser.
        """
        c = self.counts
        p = self.heartbeat_period

        if c["rgb"] or c["depth"]:
            self.ever_saw_camera = True

        if c["pairs"] == 0:
            if c["rgb"] == 0 and c["depth"] == 0:
                if not self.ever_saw_camera:
                    # Never received anything: the OAK-D does not stream on the dock,
                    # which is the overwhelmingly common cause of a cold start.
                    self.get_logger().warn(
                        f"no camera data at all in {p:.0f}s — IS THE ROBOT DOCKED? "
                        f"The OAK-D does not publish while docked. Undock with:\n"
                        f"    ros2 action send_goal /undock "
                        f"irobot_create_msgs/action/Undock \"{{}}\"\n"
                        f"  (if it is already undocked, check: "
                        f"ros2 topic hz {RGB_TOPIC})"
                    )
                else:
                    # It streamed before, so the camera and the dock are fine and
                    # the link is the suspect, not the robot.
                    self.get_logger().warn(
                        f"camera stream STALLED — it was working, now nothing for "
                        f"{p:.0f}s. Not the dock. Usually Wi-Fi: close RViz camera "
                        f"displays and rqt_image_view, and move nearer the robot."
                    )
            elif c["depth"] == 0:
                self.get_logger().warn(
                    f"RGB arriving ({c['rgb']} msgs) but NO depth in {p:.0f}s"
                    + (". Check i_pipeline_type: RGBD on the camera."
                       if not self.ever_saw_depth
                       else " — depth was working, so this is the link dropping the "
                            "bigger (355 KB) depth frames first. Reduce other traffic.")
                )
            elif c["rgb"] == 0:
                self.get_logger().warn(
                    f"depth arriving ({c['depth']} msgs) but NO RGB in {p:.0f}s. "
                    f"Check {RGB_TOPIC}."
                )
            else:
                # With a healthy link both streams are dense and pairing is easy.
                # When only a handful of each trickle in, they simply miss each
                # other, which is a bandwidth symptom rather than a clock one.
                cause = ("Both streams are sparse, so they are probably just missing "
                         "each other — a bandwidth problem, not a clock one."
                         if c["rgb"] + c["depth"] < 20 else
                         "Timestamps disagree by more than the 0.15 s slop. "
                         "Suspect clock skew: run fixbotclock.")
                self.get_logger().warn(
                    f"both streams arriving (rgb {c['rgb']}, depth {c['depth']}) but "
                    f"NOTHING pairs up. {cause}"
                )
        else:
            bits = [f"{c['pairs']} pairs ({c['pairs'] / p:.1f} Hz)",
                    f"rgb {c['rgb']}", f"depth {c['depth']}"]
            if self.infer_ms is not None:
                # Says which end is the bottleneck: if inference is much faster
                # than the pair rate, a heavier model is affordable.
                bits.append(f"yolo {self.infer_ms:.0f} ms "
                            f"({1000.0 / self.infer_ms:.1f} Hz capable)")
            if c["yolo"] == 0:
                bits.append("YOLO saw nothing above conf "
                            f"{self.conf} — point the camera at an object, or lower "
                            "conf_threshold")
            else:
                bits.append(f"{c['yolo']} objects")
                if c["wrong_class"]:
                    # Name them. "10 not 'chair'" leaves you unable to tell a
                    # camera pointed at a wall from one pointed at the target.
                    others = ", ".join(
                        f"{n} x{k}" if k > 1 else n
                        for n, k in self.seen.most_common(5)
                        if not self.target_class or n.lower() != self.target_class)
                    bits.append(f"{c['wrong_class']} not '{self.target_class}'"
                                + (f" (saw: {others})" if others else ""))
                if c["no_depth"]:
                    bits.append(f"{c['no_depth']} no depth return")
                if c["out_of_range"]:
                    bits.append(f"{c['out_of_range']} out of range")
                if c["tf_fail"]:
                    bits.append(f"{c['tf_fail']} tf failed")
                bits.append(f"{c['published']} published")
            self.get_logger().info("status: " + ", ".join(bits))

        for k in c:
            c[k] = 0
        self.seen.clear()

    def check_target_frame(self):
        """Track whether map coordinates are available, and say so when it changes.

        AMCL publishes no map -> odom transform until it has been given an
        initial pose, so 'map' can appear minutes after this node starts. Rather
        than make you restart, watch for it and switch over when it shows up.
        """
        if self.desired_frame != "map":
            return
        have_map = self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time())
        now = "map" if have_map else "base_link"
        if now == self.target_frame:
            return

        self.target_frame = now
        if have_map:
            self.get_logger().info(
                "'map' frame is up — now reporting detections in map coordinates."
            )
        else:
            self.get_logger().warn(
                "no 'map' frame yet — reporting in 'base_link' instead: x is metres "
                "in front of the robot, y is metres to its left. Either Nav2/AMCL is "
                "not running, or AMCL has no initial pose yet (click '2D Pose "
                "Estimate' in RViz). This node will switch to map automatically."
            )

    def on_info(self, msg):
        if self.K is None:
            self.K = dict(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])
            self.get_logger().info(
                f"intrinsics: fx={self.K['fx']:.1f} fy={self.K['fy']:.1f} "
                f"cx={self.K['cx']:.1f} cy={self.K['cy']:.1f}"
            )

    def depth_at(self, depth, u, v):
        """Median of the valid (non-zero) depth pixels around (u, v), in metres."""
        for k in (self.patch, self.patch * 2 + 1):
            r = k // 2
            h, w = depth.shape
            win = depth[max(0, v - r):min(h, v + r + 1),
                        max(0, u - r):min(w, u + r + 1)]
            vals = win[win > 0]
            # Insist on a few agreeing pixels; one stray return is not a depth.
            if vals.size >= 3:
                return float(np.median(vals)) / 1000.0
        return None

    def on_pair(self, rgb_msg, depth_msg):
        frame = cv2.imdecode(np.frombuffer(bytes(rgb_msg.data), np.uint8), cv2.IMREAD_COLOR)
        depth = decode_compressed_depth(depth_msg)
        if frame is None or depth is None:
            return

        if not self.announced:
            self.get_logger().info(
                f"receiving synced pairs — rgb {frame.shape[1]}x{frame.shape[0]}, "
                f"depth {depth.shape[1]}x{depth.shape[0]}"
            )
            self.announced = True

        K = self.K or FALLBACK_K

        # The depth image is already aligned to the RGB frame and is the same
        # size, so a box pixel indexes straight in. If that ever stops being
        # true, scale here rather than silently reading the wrong pixel.
        sx = depth.shape[1] / frame.shape[1]
        sy = depth.shape[0] / frame.shape[0]

        self.bump("pairs")

        t0 = time.time()
        results = self.model(frame, conf=self.conf, verbose=False)[0]
        ms = (time.time() - t0) * 1000.0
        # Exponential average: one slow frame should not dominate the report.
        self.infer_ms = ms if self.infer_ms is None else 0.9 * self.infer_ms + 0.1 * ms
        self.bump("yolo", len(results.boxes))
        annotated = results.plot()
        markers = MarkerArray()
        seen = []

        for box in results.boxes:
            name = self.model.names[int(box.cls)]
            self.seen[name] += 1
            if self.target_class and name.lower() != self.target_class:
                self.bump("wrong_class")
                continue
            conf = float(box.conf)

            x1, y1, x2, y2 = box.xyxy[0].tolist()
            u = int(round((x1 + x2) / 2.0))
            v = int(round((y1 + y2) / 2.0))

            Z = self.depth_at(depth, int(u * sx), int(v * sy))
            if Z is None:
                self.bump("no_depth")
                self.get_logger().warn(f"{name} {conf:.2f}: no depth return at box centre")
                continue
            if not (self.min_depth <= Z <= self.max_depth):
                self.bump("out_of_range")
                self.get_logger().warn(
                    f"{name} {conf:.2f}: {Z:.2f} m is outside the trusted "
                    f"{self.min_depth}-{self.max_depth} m stereo range — ignoring"
                )
                continue

            # Pinhole back-projection, in the optical frame convention:
            # x right, y down, z forward along the lens axis.
            X = (u - K["cx"]) * Z / K["fx"]
            Y = (v - K["cy"]) * Z / K["fy"]

            pt = PointStamped()
            pt.header.stamp = rgb_msg.header.stamp
            pt.header.frame_id = depth_msg.header.frame_id or "oakd_rgb_camera_optical_frame"
            pt.point.x, pt.point.y, pt.point.z = X, Y, Z

            mapped = self.to_target_frame(pt)
            if mapped is None:
                self.bump("tf_fail")
                continue

            self.bump("published")
            self.pub_point.publish(mapped)
            markers.markers.append(self.make_marker(mapped, name, conf))
            seen.append(
                f"{name} {conf:.2f} @ ({mapped.point.x:+.2f}, {mapped.point.y:+.2f}) "
                f"{Z:.2f} m out"
            )
            draw_range_label(annotated, (x1, y1, x2, y2), f"{Z:.2f} m", int(box.cls))

        if seen:
            self.get_logger().info(f"[{self.target_frame}] " + " | ".join(seen))
        if markers.markers:
            self.pub_marker.publish(markers)

        ok, jpeg = cv2.imencode(".jpg", annotated)
        if ok:
            out = CompressedImage()
            out.header = rgb_msg.header
            out.format = "jpeg"
            out.data = jpeg.tobytes()
            self.pub_img.publish(out)

    def to_target_frame(self, pt):
        """Transform a camera-frame point into the target (map) frame."""
        try:
            # Ask at the image's own timestamp: the robot may have been moving,
            # and the camera pose that produced this pixel is the one we want.
            return self.tf_buffer.transform(
                pt, self.target_frame, timeout=Duration(seconds=0.2)
            )
        except tf2_ros.ExtrapolationException:
            # Clock skew between laptop, Pi and base makes this the common
            # failure. The latest transform is close enough while stopped at a
            # waypoint, which is when the search actually takes its readings.
            try:
                fresh = PointStamped()
                fresh.header.frame_id = pt.header.frame_id
                fresh.point = pt.point
                out = self.tf_buffer.transform(
                    fresh, self.target_frame, timeout=Duration(seconds=0.2)
                )
                if not self.tf_warned:
                    self.get_logger().warn(
                        "tf at image stamp unavailable — using latest transform. "
                        "Check clock sync (run fixbotclock) if the robot is moving."
                    )
                    self.tf_warned = True
                return out
            except tf2_ros.TransformException as e:
                self.warn_tf(e)
                return None
        except tf2_ros.TransformException as e:
            self.warn_tf(e)
            return None

    def warn_tf(self, err):
        if not self.tf_warned:
            self.get_logger().warn(
                f"cannot transform into '{self.target_frame}': {err}. "
                f"Is Nav2/AMCL running and localized?"
            )
            self.tf_warned = True

    def make_marker(self, pt, name, conf):
        m = Marker()
        m.header = pt.header
        m.ns = "detections"
        m.id = self.marker_id
        self.marker_id += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = pt.point
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.g, m.color.b = 1.0, 0.35, 0.0
        m.color.a = 0.9
        m.lifetime = Duration(seconds=30.0).to_msg()
        m.text = f"{name} {conf:.2f}"
        return m


def main(args=None):
    rclpy.init(args=args)
    node = Detector3D()
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
