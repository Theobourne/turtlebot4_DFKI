#!/usr/bin/env python3
"""
search_common.py — the machinery every search algorithm shares.

A search algorithm should only have to answer one question: *where do I look
next?* Everything around that question — collecting sightings, clustering them,
deciding when the evidence is convincing, saving proof, docking, reporting — is
the same whichever strategy is driving, so it lives here.

An algorithm is a module exposing:

    NAME        short key used on the command line, e.g. "kmeans-tour"
    TITLE       one line for the menu
    DESCRIPTION a sentence or two on how it searches
    run(nav, collector, waypoints, target) -> cluster dict or None

See kmeans_waypoint_tour.py for the reference implementation, and
search_algorithms.py for the registry that lists them.
"""

import collections
import math
import os
import time

import numpy as np
import cv2
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from irobot_create_msgs.action import Dock
from nav2_simple_commander.robot_navigator import TaskResult
from tf2_ros import Buffer, TransformListener

from object_detector.start_pose import load_start_pose

# ---- settings you can change ---------------------------------------------
POINT_TOPIC = "/detections/object_point"
ANNOTATED_TOPIC = "/detections/annotated/compressed"
RESULT_TOPIC = "/search/result_marker"

DWELL_SECONDS = 3.0        # how long to stand still and look at each stop
SWEEP_STEPS = 4            # in-place turns per waypoint (4 = look all the way round)
CLUSTER_RADIUS = 0.6       # sightings closer than this are the same object, metres
MIN_SIGHTINGS = 3          # how many agreeing looks before we call it found

# Where the annotated proof-of-find images go. Untracked by git: they are run
# artefacts, not source.
FINDS_DIR = os.path.join(os.path.expanduser("~"), "turtlebot4_ws", "finds")

RETURN_TO_DOCK = True      # drive home once the search ends
DOCK_TIMEOUT = 180.0

# ---- continuous sensing ---------------------------------------------------
# With this on, the robot keeps looking while it drives. Detections gathered in
# motion are treated as *cues*, not evidence: motion blur, noisier stereo and tf
# error that grows with speed make them good enough to say "something over
# there" and not good enough to answer with. A cue interrupts the drive; the
# answer only ever comes from a standstill.
CUE_MIN_SIGHTINGS = 2      # clustered sightings in motion before interrupting
MAX_VERIFICATIONS = 3      # per run, so a flapping detector cannot eat the battery
BLACKLIST_RADIUS = 0.8     # a rejected candidate will not re-trigger within this

# How far to sidestep for a second viewpoint. Fixed distances are the wrong
# shape: what matters is the angle subtended at the object, so the offset scales
# with range to hold that angle roughly constant. At 2 m this asks for ~0.42 m,
# at 6 m it saturates at 1.0 m.
VERIFY_PARALLAX_DEG = 12.0
VERIFY_MIN_BASELINE = 0.40
VERIFY_MAX_BASELINE = 1.00

# Nav2's xy_goal_tolerance is 0.25 m, so a commanded sidestep of 0.4 m can land
# anywhere from 0.15 to 0.65 m out. Never trust the commanded offset — measure
# what was actually achieved from tf, and judge independence on that.
VERIFY_MIN_SPREAD = 0.25   # metres between observing poses to count as a second view

# How many recent annotated frames to keep so the proof image can be chosen
# rather than whatever happens to be on screen at the end. ~180 KB each.
PROOF_BUFFER = 40
STAMP_TOLERANCE = 0.02     # seconds; frames and points share the image's stamp
# --------------------------------------------------------------------------

DEFAULT_WAYPOINTS = [
    {"x": 1.0, "y": 0.0, "yaw": 0.0},
    {"x": 2.0, "y": 1.0, "yaw": 1.57},
    {"x": 1.0, "y": 2.0, "yaw": 3.14},
]


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TargetFound(Exception):
    """Raised when a cue is verified mid-drive, carrying the confirmed cluster.

    Control flow via exception, deliberately: it lets the interrupt unwind from
    inside drive_to() all the way to the runner without every search strategy
    having to know the mechanism exists. Strategies stay written as if the robot
    simply drove where they asked.
    """

    def __init__(self, cluster):
        super().__init__("target found and verified")
        self.cluster = cluster


class SightingCollector(Node):
    """Listens for detect_3d's map-frame points and groups them by object."""

    def __init__(self):
        super().__init__("search_collector")
        # Each cluster: {"pts": [(x,y,z)], "centre": (x,y,z),
        #                "poses": [(x,y) or None], "moving": [bool]}
        # The observing pose travels with every sighting. That is what lets
        # verification ask "have I already seen this from two places?" instead
        # of always driving somewhere to find out.
        self.clusters = []
        self.create_subscription(PointStamped, POINT_TOPIC, self.on_point, 10)
        self.pub_marker = self.create_publisher(MarkerArray, RESULT_TOPIC, 10)
        self.collecting = True
        self.moving = False          # tags incoming sightings as cue vs evidence
        self.continuous = False      # set by the runner from a parameter
        self.blacklist = []          # centres of candidates that failed verification
        self.verifications = 0

        # A short rolling buffer of annotated frames, kept by timestamp.
        #
        # Not just "the latest one": detect_3d publishes the point before the
        # annotated image for the same frame, so at the moment a sighting
        # arrives the newest annotated frame is usually the PREVIOUS one. And by
        # the time the find is saved the robot has turned to face the target, so
        # the newest frame then is a motion-blurred one from mid-spin — which is
        # exactly the useless picture this replaces. Keeping a buffer lets the
        # proof image be *chosen* afterwards, by matching the stamps of the
        # sightings that actually convinced us.
        self.latest_annotated = None
        self.recent_annotated = collections.deque(maxlen=PROOF_BUFFER)
        self.create_subscription(
            CompressedImage, ANNOTATED_TOPIC, self.on_annotated, 10
        )
        self.dock_client = ActionClient(self, Dock, "dock")

        # Used by algorithms that plan relative to where the robot actually is,
        # rather than relative to the last waypoint they aimed at.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    @staticmethod
    def _stamp_seconds(header):
        return header.stamp.sec + header.stamp.nanosec / 1e9

    def on_annotated(self, msg):
        self.latest_annotated = msg
        self.recent_annotated.append((self._stamp_seconds(msg.header), msg))

    def observer_pose(self, stamp):
        """Where the robot was when a sighting's image was taken, or None.

        Looked up at the image's own timestamp rather than "now", so a sighting
        that arrived late over Wi-Fi is still attributed to the pose that
        produced it. Falls back to the latest transform, then gives up — a
        sighting without a pose still counts, it just cannot contribute
        parallax.
        """
        for t in (rclpy.time.Time.from_msg(stamp), rclpy.time.Time()):
            try:
                tr = self.tf_buffer.lookup_transform(
                    "map", "base_link", t, timeout=Duration(seconds=0.05))
                return (tr.transform.translation.x, tr.transform.translation.y)
            except Exception:
                continue
        return None

    def current_pose(self, fallback=(0.0, 0.0, 0.0)):
        """(x, y, yaw) in the map frame, or `fallback` if tf cannot say."""
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time(),
                    timeout=Duration(seconds=0.1))
                return (t.transform.translation.x, t.transform.translation.y,
                        quat_to_yaw(t.transform.rotation))
            except Exception:
                continue
        self.get_logger().warn("could not read the robot pose from tf")
        return fallback

    def current_xy(self, fallback=(0.0, 0.0)):
        """Where the robot is in the map frame, or `fallback` if tf cannot say.

        Falling back rather than raising is deliberate: a strategy that cannot
        read the pose should still search, just less cleverly. Losing the tour
        order is a far smaller failure than losing the run.
        """
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time(),
                    timeout=Duration(seconds=0.1))
                return (t.transform.translation.x, t.transform.translation.y)
            except Exception:
                continue
        self.get_logger().warn("could not read the robot pose from tf")
        return fallback

    @staticmethod
    def _sharpness(frame):
        """Variance of the Laplacian — the standard cheap blur score.

        A blurred image has little high-frequency content, so the second
        derivative is small everywhere and its variance collapses. Higher is
        sharper.

        Only meaningful BETWEEN FRAMES OF THE SAME SCENE, which is how it is
        used here — all candidates come from one cluster. Across scenes it says
        more about how much texture is in view than about focus: a blurred photo
        of a doorway outscores a sharp photo of blank carpet.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var())

    def pick_proof_frame(self, cluster):
        """The sharpest annotated frame that actually produced a sighting.

        Candidates are matched to the cluster's sightings by timestamp, so every
        one of them is a frame the detector found the target in. Sightings taken
        at a standstill are preferred: a frame captured while driving carries
        motion blur no scoring can undo.

        Decoding happens here, once at the end, rather than per sighting during
        the search — the CPU during a run belongs to Nav2's controller.
        """
        if not cluster or not cluster.get("stamps"):
            return None

        for moving_ok in (False, True):
            wanted = [s for s, m in zip(cluster["stamps"], cluster["moving"])
                      if m == moving_ok or moving_ok]
            best, best_score = None, -1.0
            for stamp, msg in self.recent_annotated:
                if not any(abs(stamp - w) < STAMP_TOLERANCE for w in wanted):
                    continue
                frame = cv2.imdecode(
                    np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                score = self._sharpness(frame)
                if score > best_score:
                    best, best_score = frame, score
            if best is not None:
                self.get_logger().info(
                    f"proof image: sharpest of the frames that saw it "
                    f"({'stationary' if not moving_ok else 'in motion'}, "
                    f"focus score {best_score:.0f})")
                return best
        return None

    def save_find(self, target, cluster, n):
        """Write the annotated frame that proves the find, and return its path."""
        centre = cluster["centre"]
        frame = self.pick_proof_frame(cluster)
        if frame is None:
            # Nothing matched — fall back to the old behaviour rather than
            # producing no evidence at all, but say that is what happened.
            self.get_logger().warn(
                "no matching annotated frame — falling back to the latest one, "
                "which may be blurred")
            if self.latest_annotated is None:
                self.get_logger().warn("no annotated frame to save")
                return None
            frame = cv2.imdecode(
                np.frombuffer(bytes(self.latest_annotated.data), np.uint8),
                cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("could not decode the annotated frame")
            return None

        # Caption it with the answer, so the file stands on its own later.
        x, y, z = centre
        caption = f"{target}  map ({x:+.2f}, {y:+.2f}, {z:+.2f})  {n} sightings"
        cv2.rectangle(frame, (0, frame.shape[0] - 34), (frame.shape[1], frame.shape[0]),
                      (0, 0, 0), -1)
        cv2.putText(frame, caption, (10, frame.shape[0] - 11),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

        os.makedirs(FINDS_DIR, exist_ok=True)
        path = os.path.join(
            FINDS_DIR, f"{time.strftime('%Y%m%d_%H%M%S')}_{target.replace(' ', '_')}.jpg"
        )
        return path if cv2.imwrite(path, frame) else None

    def dock(self):
        """Drive back onto the dock. Returns True if the robot ends up docked."""
        if not self.dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("no /dock action server — cannot return to dock")
            return False
        self.get_logger().info("returning to dock...")
        goal = self.dock_client.send_goal_async(Dock.Goal())
        rclpy.spin_until_future_complete(self, goal, timeout_sec=20.0)
        if not goal.done() or not goal.result().accepted:
            self.get_logger().warn("dock goal rejected")
            return False
        result = goal.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=DOCK_TIMEOUT)
        if not result.done():
            self.get_logger().warn("docking timed out")
            return False
        return bool(result.result().result.is_docked)

    def on_point(self, msg):
        if not self.collecting:
            return
        p = (msg.point.x, msg.point.y, msg.point.z)
        pose = self.observer_pose(msg.header.stamp)
        stamp = self._stamp_seconds(msg.header)
        for c in self.clusters:
            cx, cy, _ = c["centre"]
            if math.hypot(p[0] - cx, p[1] - cy) < CLUSTER_RADIUS:
                c["pts"].append(p)
                c["poses"].append(pose)
                c["moving"].append(self.moving)
                c["stamps"].append(stamp)
                c["centre"] = tuple(sum(v) / len(c["pts"]) for v in zip(*c["pts"]))
                return
        self.clusters.append({"pts": [p], "centre": p, "poses": [pose],
                              "moving": [self.moving], "stamps": [stamp]})

    def drain(self):
        """Throw away anything queued while the robot was driving."""
        was, self.collecting = self.collecting, False
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.0)
        self.collecting = was

    def look(self, seconds):
        """Stand still and accept sightings. These are evidence, not cues.

        Drains first, always. Anything already queued was captured before the
        robot came to rest, and counting it here would quietly relabel an
        in-motion sighting as evidence — the one thing the cue/evidence split
        exists to prevent.
        """
        self.drain()
        was_moving, self.moving = self.moving, False
        self.collecting = True
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.moving = was_moving

    def best(self):
        """The cluster with the most agreeing sightings, if it is convincing."""
        if not self.clusters:
            return None
        c = max(self.clusters, key=lambda c: len(c["pts"]))
        return c if len(c["pts"]) >= MIN_SIGHTINGS else None

    # ---- continuous sensing ------------------------------------------------

    @staticmethod
    def stationary(cluster):
        """Sightings taken at a standstill. Only these can answer the question."""
        return sum(1 for m in cluster["moving"] if not m)

    @staticmethod
    def spread(cluster):
        """Widest separation between any two poses this cluster was seen from.

        Cue poses count towards this. A sighting taken in motion is too coarse
        to *be* the answer, but it agreed with the cluster to within
        CLUSTER_RADIUS from somewhere else in the room, and that agreement is
        exactly the corroboration parallax is meant to provide.
        """
        poses = [p for p in cluster["poses"] if p is not None]
        return max((math.hypot(a[0] - b[0], a[1] - b[1])
                    for i, a in enumerate(poses) for b in poses[i + 1:]),
                   default=0.0)

    def is_false_alarm(self, centre):
        return any(math.hypot(centre[0] - x, centre[1] - y) < BLACKLIST_RADIUS
                   for x, y in self.blacklist)

    def reject(self, cluster):
        """Remember a candidate that failed verification, so it stops nagging."""
        self.blacklist.append((cluster["centre"][0], cluster["centre"][1]))

    def cue(self):
        """A candidate worth interrupting the drive for, or None."""
        if self.verifications >= MAX_VERIFICATIONS:
            return None
        for c in self.clusters:
            if len(c["pts"]) < CUE_MIN_SIGHTINGS:
                continue
            if self.is_false_alarm(c["centre"]):
                continue
            return c
        return None

    def publish_result(self, centre, label):
        markers = MarkerArray()
        ball = Marker()
        ball.header.frame_id = "map"
        ball.header.stamp = self.get_clock().now().to_msg()
        ball.ns = "search_result"
        ball.id = 0
        ball.type = Marker.SPHERE
        ball.action = Marker.ADD
        ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = centre
        ball.pose.orientation.w = 1.0
        ball.scale.x = ball.scale.y = ball.scale.z = 0.4
        ball.color.g, ball.color.a = 1.0, 0.9
        markers.markers.append(ball)

        text = Marker()
        text.header = ball.header
        text.ns = "search_result"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = centre[0]
        text.pose.position.y = centre[1]
        text.pose.position.z = centre[2] + 0.4
        text.pose.orientation.w = 1.0
        text.scale.z = 0.3
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = label
        markers.markers.append(text)

        self.pub_marker.publish(markers)


def load_waypoints(node):
    node.declare_parameter("waypoints_file", "")
    path = node.get_parameter("waypoints_file").value
    if path and os.path.exists(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        wps = data.get("waypoints", [])
        if wps:
            node.get_logger().info(f"loaded {len(wps)} waypoints from {path}")
            return wps
    node.get_logger().warn(
        "no waypoints_file given — using placeholder waypoints. "
        "Edit config/search_waypoints.yaml for your room."
    )
    return DEFAULT_WAYPOINTS


def make_pose(nav, wp):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(wp["x"])
    p.pose.position.y = float(wp["y"])
    qx, qy, qz, qw = yaw_to_quat(float(wp.get("yaw", 0.0)))
    p.pose.orientation.x, p.pose.orientation.y = qx, qy
    p.pose.orientation.z, p.pose.orientation.w = qz, qw
    return p


# ---- the two moves every waypoint-based strategy is built from -------------

def _wait(nav):
    while not nav.isTaskComplete():
        time.sleep(0.2)
    return nav.getResult() == TaskResult.SUCCEEDED


def _goto(nav, collector, x, y, yaw):
    collector.moving = True
    nav.goToPose(make_pose(nav, {"x": x, "y": y, "yaw": yaw}))
    ok = _wait(nav)
    collector.moving = False
    return ok


def _face(nav, collector, target):
    """Turn on the spot until the candidate is centred in the camera."""
    x, y, yaw = collector.current_pose()
    want = math.atan2(target[1] - y, target[0] - x)
    delta = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
    if abs(delta) < math.radians(5):
        return
    collector.moving = True
    nav.spin(spin_dist=delta, time_allowance=20)
    _wait(nav)
    collector.moving = False


def verify(nav, collector, cand):
    """Stop and decide whether a cue is really the target.

    Returns the cluster if confirmed, None if it was a false alarm. The rule is
    that the answer needs agreeing sightings taken at a standstill, seen from
    two places far enough apart to be independent looks rather than the same
    look sampled twice.
    """
    collector.verifications += 1
    cx, cy = cand["centre"][0], cand["centre"][1]
    collector.get_logger().info(
        f"  cue at ({cx:+.2f}, {cy:+.2f}) — stopping to check "
        f"({collector.verifications}/{MAX_VERIFICATIONS})")

    _face(nav, collector, (cx, cy))
    collector.look(DWELL_SECONDS)

    # The approach may already have supplied the parallax: if the drive passed
    # the candidate obliquely, its cue sightings came from well-separated poses
    # and there is nothing left to prove by moving again.
    if (collector.stationary(cand) >= MIN_SIGHTINGS
            and collector.spread(cand) >= VERIFY_MIN_SPREAD):
        collector.get_logger().info(
            f"  confirmed from the approach — {collector.spread(cand):.2f} m "
            f"between viewpoints, no extra move needed")
        cand["verified"] = {"baseline": collector.spread(cand), "sidestepped": False}
        return cand

    # Otherwise take a second viewpoint. The offset scales with range to hold
    # the parallax angle roughly constant, and both sides are tried because the
    # first may have no room to plan into.
    x, y, _ = collector.current_pose()
    d = math.hypot(cx - x, cy - y)
    if d < 1e-3:
        collector.reject(cand)
        return None
    need = min(max(math.tan(math.radians(VERIFY_PARALLAX_DEG)) * d,
                   VERIFY_MIN_BASELINE), VERIFY_MAX_BASELINE)
    ux, uy = (cx - x) / d, (cy - y) / d

    collector.get_logger().info(
        f"  target is {d:.2f} m away — sidestepping {need:.2f} m for a second view")
    moved = False
    for sign in (1.0, -1.0):
        gx, gy = x - uy * sign * need, y + ux * sign * need
        if _goto(nav, collector, gx, gy, math.atan2(cy - gy, cx - gx)):
            moved = True
            break
        collector.get_logger().warn("  that side did not plan — trying the other")

    if moved:
        # Never assume the commanded offset: Nav2's goal tolerance is 0.25 m,
        # more than half of the smallest sidestep we ever ask for.
        nx, ny, _ = collector.current_pose()
        collector.get_logger().info(
            f"  moved {math.hypot(nx - x, ny - y):.2f} m (asked for {need:.2f})")
        collector.look(DWELL_SECONDS)

    n, baseline = collector.stationary(cand), collector.spread(cand)
    if n >= MIN_SIGHTINGS and baseline >= VERIFY_MIN_SPREAD:
        collector.get_logger().info(
            f"  confirmed — {n} sightings from viewpoints {baseline:.2f} m apart")
        cand["verified"] = {"baseline": baseline, "sidestepped": True}
        return cand

    if n >= 2 * MIN_SIGHTINGS:
        # Plenty of agreeing looks but no usable baseline. Accept it rather than
        # throw away a probable find, and say so, because it is weaker evidence
        # than the line above and the summary should not pretend otherwise.
        collector.get_logger().warn(
            f"  accepting on weight of evidence — {n} sightings, but only "
            f"{baseline:.2f} m of parallax")
        cand["verified"] = {"baseline": baseline, "sidestepped": True}
        return cand

    collector.get_logger().info(
        f"  not confirmed ({n} sightings, {baseline:.2f} m parallax) — "
        f"ignoring this spot and carrying on")
    collector.reject(cand)
    return None


def go_home(nav, collector):
    """Drive back to the recorded start pose. True if the robot got there.

    The Dock action only looks for the dock in the robot's immediate
    surroundings — called from across the room it simply fails, which is what
    left the robot stranded mid-floor after a successful find. Nav2 already
    knows the way home, so let it cover the distance and leave docking with the
    last metre, which is all it can actually do.

    A failure here is reported and not fatal: docking from wherever the robot
    ended up is still worth attempting, and might work if it happens to be near.
    """
    home = load_start_pose()
    if not home:
        collector.get_logger().warn(
            "no saved start pose — docking from here, which usually fails if "
            "the dock is far away. Record one with 'mission'.")
        return False

    x, y, yaw = home["x"], home["y"], home["yaw"]
    collector.get_logger().info(f"driving home to ({x:+.2f}, {y:+.2f}) before docking...")
    if _goto(nav, collector, x, y, yaw):
        collector.get_logger().info("back at the start pose.")
        return True

    collector.get_logger().warn(
        "could not drive home — trying to dock from here anyway.")
    return False


def drive_to(nav, collector, wp, label=""):
    """Send the robot to one waypoint. True if it got there.

    An unreachable waypoint is reported and skipped rather than aborting the
    run: one blocked corner should not end the search.

    With continuous sensing on, the robot watches while it drives and will
    interrupt itself to check anything promising, raising TargetFound if the
    check succeeds. Search strategies do not have to know any of this happened.
    """
    collector.get_logger().info(
        f"--- {label}({wp['x']:.2f}, {wp['y']:.2f}) ---" if label
        else f"--- ({wp['x']:.2f}, {wp['y']:.2f}) ---"
    )
    pose = make_pose(nav, wp)
    nav.goToPose(pose)
    collector.moving = True

    while not nav.isTaskComplete():
        if not collector.continuous:
            # Blind driving: nothing is spun, so nothing arriving now is
            # recorded, and drain() clears the backlog on arrival.
            time.sleep(0.2)
            continue

        rclpy.spin_once(collector, timeout_sec=0.1)
        cand = collector.cue()
        if cand is None:
            continue

        nav.cancelTask()
        hit = verify(nav, collector, cand)
        if hit is not None:
            collector.moving = False
            raise TargetFound(hit)
        # Resume the interrupted leg.
        nav.goToPose(pose)
        collector.moving = True

    collector.moving = False
    if nav.getResult() != TaskResult.SUCCEEDED:
        collector.get_logger().warn("could not reach that waypoint — skipping")
        return False

    # Detections that arrived while driving are ignored: the robot was moving,
    # so their tf lookups are the least trustworthy ones.
    collector.drain()
    return True


def sweep_in_place(nav, collector, steps=SWEEP_STEPS, dwell=DWELL_SECONDS):
    """Stand still and turn on the spot, looking. Returns a cluster or None.

    Checked between steps rather than only at the end, so the robot can stop
    turning the moment the evidence is good enough.
    """
    for step in range(max(1, steps)):
        collector.get_logger().info(f"    looking ({step + 1}/{steps})...")
        collector.look(dwell)

        hit = collector.best()
        if hit:
            return hit

        if steps > 1 and step < steps - 1:
            nav.spin(spin_dist=2.0 * math.pi / steps, time_allowance=15)
            while not nav.isTaskComplete():
                time.sleep(0.2)
            collector.drain()
    return None
