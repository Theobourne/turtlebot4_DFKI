#!/usr/bin/env python3
"""
mission.py — the front door. Ask what to look for, check the robot is ready,
then run the search.

    ros2 run object_detector mission

This is the one command for a demo. It asks for a target object, then runs
through the things that actually go wrong on this setup before committing the
robot to a search:

  * is the robot docked? (the OAK-D does not stream on the dock)
  * are RGB and depth both arriving, and pairing up?
  * is there a 'map' frame, i.e. has AMCL been given an initial pose?
  * is Nav2 accepting goals?
  * do the waypoints load?

Each check names its own fix, and the docked check can undock for you. Anything
still wrong is reported before the robot moves, rather than as silence twenty
seconds into a run.
"""

import math
import os
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy)

import tf2_ros
from sensor_msgs.msg import BatteryState, CompressedImage, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose

from object_detector.start_pose import load_start_pose, save_start_pose
from std_srvs.srv import Empty, Trigger

from object_detector.detect_3d import RGB_TOPIC, DEPTH_TOPIC

WS = os.path.join(os.path.expanduser("~"), "turtlebot4_ws")
DEFAULT_WAYPOINTS = os.path.join(WS, "config", "search_waypoints.yaml")
DEFAULT_MAP = os.path.join(WS, "maps", "room_a.yaml")
DEFAULT_NAV2_PARAMS = os.path.join(WS, "config", "nav2_lab.yaml")
START_POSE_FILE = os.path.join(WS, "config", "start_pose.yaml")
LOG_DIR = os.path.join(WS, "logs", "mission")

# AMCL's starting uncertainty. Loose enough that a hand-recorded dock pose off
# by a few centimetres still converges once the laser sees the walls.
INITIAL_COVARIANCE = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853]

# COCO classes worth searching for indoors. The model knows all 80; these are
# just the ones that make sense in a lab, offered as a menu so you do not have
# to remember the exact spelling YOLO expects.
SUGGESTED = [
    "chair", "person", "bottle", "laptop", "tv", "backpack", "book",
    "cup", "keyboard", "mouse", "potted plant", "couch", "dining table",
    "handbag", "cell phone", "suitcase", "clock", "vase",
]

OK, WARN, FAIL = "\033[92m  ok  \033[0m", "\033[93m warn \033[0m", "\033[91m FAIL \033[0m"
BOLD, DIM, RESET = "\033[1m", "\033[2m", "\033[0m"


def snap_confined():
    """True if we are inside a snap's environment (e.g. VS Code's terminal).

    The VS Code snap exports its confined environment into every terminal it
    spawns. System GUI binaries launched from there pick up the snap's runtime
    and abort:

        rviz2: symbol lookup error: /snap/core20/current/lib/.../libpthread.so.0:
        undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE

    Verified by launching rviz2 with a scrubbed environment, where the error
    does not occur. There is no reliable way to un-confine a child, so the fix
    is to run from an ordinary terminal instead.
    """
    return any(k.startswith("SNAP") for k in os.environ)


def _gui_safe_env():
    """The environment, minus OpenCV's Qt overrides.

    Importing cv2 (which this package does, for the detector) sets
    QT_QPA_PLATFORM_PLUGIN_PATH and QT_QPA_FONTDIR to OpenCV's own bundled Qt
    plugins. Any Qt GUI we then launch inherits them, tries to load the wrong
    xcb plugin and aborts:

        qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in
        ".../site-packages/cv2/qt/plugins" even though it was found.

    That kills RViz. Drop the variables only when they point into cv2, so a
    deliberate Qt setup of your own is left alone.
    """
    env = os.environ.copy()
    for var in ("QT_QPA_PLATFORM_PLUGIN_PATH", "QT_QPA_FONTDIR"):
        if "cv2" in env.get(var, ""):
            env.pop(var, None)
    return env


class StackManager:
    """Start localization and Nav2 as background children, if they are not up.

    They are ordinary long-running processes, so there is nothing that actually
    requires a terminal each — the usual one-per-terminal habit is just so you
    can read their logs. Here their output goes to files under logs/mission/
    instead, and they are stopped on the way out.

    The rule this class follows: only ever stop what it started. If you already
    had Nav2 running in another terminal, it is left alone, because killing a
    stack someone else is watching is a nasty surprise.
    """

    def __init__(self):
        self.started = []  # (name, Popen, logfile)
        os.makedirs(LOG_DIR, exist_ok=True)

    def start(self, name, cmd, keep=False):
        """Start a child. keep=True means leave it running when we shut down."""
        path = os.path.join(LOG_DIR, f"{name}.log")
        log = open(path, "w")
        log.write(f"$ {' '.join(cmd)}\n\n")
        log.flush()
        # Its own process group, so we can signal the whole launch tree — a
        # ros2 launch spawns several nodes and killing only the parent orphans
        # them, which then quietly hold the ports and confuse the next run.
        proc = subprocess.Popen(
            cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            env=_gui_safe_env(),
        )
        self.started.append((name, proc, path, keep))
        print(f"  {DIM}started {name} (pid {proc.pid}) — log: {path}{RESET}")
        return proc

    @staticmethod
    def _group_alive(pgid):
        """True while any process in the group is still running.

        Deliberately not `killpg(pgid, 0)`: a process that has already exited
        stays in the table as a zombie until it is reaped, and signal 0 succeeds
        against a zombie. That reads as "still alive" and makes the shutdown
        escalate all the way to SIGKILL against processes that already died.
        Reading /proc lets us skip state 'Z'.
        """
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/stat") as f:
                    # comm can contain spaces and brackets, so split after the ')'.
                    fields = f.read().rsplit(")", 1)[1].split()
                state, pgrp = fields[0], int(fields[2])
            except (OSError, IndexError, ValueError):
                continue
            if pgrp == pgid and state != "Z":
                return True
        return False

    def _signal_group(self, proc, pgid, sig, timeout):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True
        end = time.time() + timeout
        while time.time() < end:
            proc.poll()  # reap the parent so it stops counting as a zombie
            if not self._group_alive(pgid):
                return True
            time.sleep(0.2)
        return not self._group_alive(pgid)

    def stop_all(self):
        for name, proc, _, keep in reversed(self.started):
            if keep and proc.poll() is None:
                # RViz is the case for this: killing it the moment the search
                # finishes would take the result markers off screen exactly when
                # you want to look at them.
                print(f"  {DIM}leaving {name} running — close it yourself.{RESET}")
                continue
            try:
                pgid = os.getpgid(proc.pid)
            except ProcessLookupError:
                continue
            if not self._group_alive(pgid):
                continue

            print(f"  {DIM}stopping {name}...{RESET}")
            # Escalate against the whole group, not just the parent. A launch
            # parent can exit on SIGINT while the nodes it spawned keep running,
            # and those orphans hold onto DDS ports and confuse the next run.
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                if self._signal_group(proc, pgid, sig, 8 if sig == signal.SIGINT else 4):
                    break
                print(f"  {DIM}  {name} ignored {sig.name}, escalating...{RESET}")
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
        self.started.clear()

    def died(self):
        """Any child that exited on its own — i.e. failed to come up."""
        return [(n, p, path) for n, p, path, _ in self.started if p.poll() is not None]


class Preflight(Node):
    """Passive checks against the live system, plus the one action we may need."""

    def __init__(self):
        super().__init__("mission_preflight")
        self.rgb = 0
        self.depth = 0
        self.scans = 0
        self.docked = None   # None = never heard from /dock_status
        self.battery = None  # fraction 0.0-1.0, as published

        self.create_subscription(CompressedImage, RGB_TOPIC, self._rgb, 10)
        self.create_subscription(CompressedImage, DEPTH_TOPIC, self._depth, 10)
        self.create_subscription(DockStatus, "/dock_status", self._dock, 10)
        # /battery_state publishes only about every 5 s, so a plain volatile
        # subscriber has to sit and wait for the next one (measured: first
        # message at 2.6 s). The publisher is TRANSIENT_LOCAL, so matching it
        # delivers the last value on connection instead — measured 0.5 s.
        battery_qos = QoSProfile(depth=1)
        battery_qos.reliability = ReliabilityPolicy.RELIABLE
        battery_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            BatteryState, "/battery_state", self._battery, battery_qos
        )
        # The lidar publishes BEST_EFFORT, so this needs the sensor profile —
        # a RELIABLE subscriber here receives nothing at all.
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.undock_client = ActionClient(self, Undock, "undock")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        # Listening to the same topic we publish on lets the pose wait tell
        # "you have not clicked yet" apart from "you clicked and AMCL is stuck",
        # which need completely different things from you.
        self.pose_clicks = 0
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._pose_click, 10
        )

    def _pose_click(self, _m):
        self.pose_clicks += 1

    # --- readiness probes ---------------------------------------------------
    def has_map_frame(self):
        return self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time())

    def localization_up(self, settle=0.0):
        """map_server and amcl both present, and /map actually published.

        `settle` spins first: the ROS graph is not populated the instant a node
        starts, and a false "not running" here would start a duplicate stack.

        NOTE: this answers "is a localization stack running?", which is the
        question worth asking before starting a second one. It does NOT mean
        localization works — see localization_active().
        """
        if settle:
            self.listen(settle)
        names = self.get_node_names()
        if not ("map_server" in names and "amcl" in names):
            return False
        return self.count_publishers("/map") > 0

    # ---- lifecycle ---------------------------------------------------------
    # map_server and amcl are lifecycle nodes: they exist in the graph, and
    # advertise /map, long before they can do anything. Presence is therefore a
    # useless readiness test — on 2026-08-18 the lifecycle manager stalled with
    # "failed to send response to /map_server/change_state (timeout)", leaving
    # map_server inactive and amcl unconfigured, while every presence check
    # passed in one second. The initial pose was then published to a node that
    # was not listening, and the run failed thirty seconds later complaining
    # about a missing 'map' frame. Ask the nodes what state they are in.

    def lifecycle_state(self, node_name, timeout=6.0):
        """The node's lifecycle label ('active', 'inactive', ...) or None."""
        from lifecycle_msgs.srv import GetState
        cli = self.create_client(GetState, f"/{node_name}/get_state")
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                return None
            fut = cli.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            if not fut.done() or fut.result() is None:
                return None
            return fut.result().current_state.label
        finally:
            self.destroy_client(cli)

    def _transition(self, node_name, transition_id, timeout=25.0):
        from lifecycle_msgs.srv import ChangeState
        cli = self.create_client(ChangeState, f"/{node_name}/change_state")
        try:
            if not cli.wait_for_service(timeout_sec=10.0):
                return False
            req = ChangeState.Request()
            req.transition.id = transition_id
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return bool(fut.done() and fut.result() is not None
                        and fut.result().success)
        finally:
            self.destroy_client(cli)

    def localization_active(self):
        return all(self.lifecycle_state(n) == "active"
                   for n in LOCALIZATION_NODES)

    def localization_states(self):
        """e.g. 'map_server inactive, amcl unconfigured' — for progress lines."""
        return ", ".join(f"{n} {self.lifecycle_state(n) or 'unknown'}"
                         for n in LOCALIZATION_NODES)

    def force_localization_active(self):
        """Drive the transitions the lifecycle manager failed to complete.

        Configuring and activating by hand is exactly what the manager would
        have done; doing it here turns a stalled bringup from a dead run into a
        few seconds of delay.
        """
        CONFIGURE, ACTIVATE = 1, 3
        ok = True
        for name in LOCALIZATION_NODES:
            state = self.lifecycle_state(name)
            if state == "active":
                continue
            if state == "unconfigured":
                if not self._transition(name, CONFIGURE):
                    print(f"  {WARN} could not configure {name}.")
                    ok = False
                    continue
            if self.lifecycle_state(name) == "inactive":
                if not self._transition(name, ACTIVATE):
                    print(f"  {WARN} could not activate {name}.")
                    ok = False
        return ok and self.localization_active()

    def nav2_up(self, timeout=1.0):
        return self.nav_client.wait_for_server(timeout_sec=timeout)

    def rviz_up(self):
        return any("rviz" in n.lower() for n in self.get_node_names())

    def nodes_present(self, expected):
        """How many of `expected` are in the graph — e.g. '5/8 nodes'."""
        names = set(self.get_node_names())
        return f"{sum(1 for n in expected if n in names)}/{len(expected)} nodes"

    def settle(self, predicate, timeout):
        """Spin until predicate() is true, quietly. Returns the final result.

        Discovery is not instant and scales with the size of the graph: with the
        robot, Nav2, localization and RViz all up, a freshly created node can
        need several seconds before it sees them. Sampling once and acting on
        the answer is how you end up starting a second copy of a stack that was
        already running.
        """
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return predicate()

    def wait_for(self, predicate, timeout, label, progress=None):
        """Spin until predicate() is true, showing a dot per second.

        `progress` is an optional callable returning a short string, printed
        every few seconds. A row of dots proves we are alive but not that
        anything is happening — during a 30 s Nav2 bringup that is
        indistinguishable from a hang. A count of nodes that have appeared so
        far answers the only question you actually have: is it getting there?
        """
        start = time.time()
        end = start + timeout
        print(f"  {DIM}waiting for {label} ", end="", flush=True)
        last_dot = last_progress = time.time()
        while time.time() < end and rclpy.ok():
            if predicate():
                print(f" up ({time.time() - start:.0f}s).{RESET}")
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.time()
            if progress and now - last_progress >= 5.0:
                print(f" [{progress()}]", end="", flush=True)
                last_progress = last_dot = now
            elif now - last_dot >= 1.0:
                print(".", end="", flush=True)
                last_dot = now
        print(f" timed out after {timeout:.0f}s.{RESET}")
        return False

    def wait_for_pose(self, timeout=300.0, report_every=10.0):
        """Wait for AMCL to localize, saying what is missing while it waits.

        A bare 'waiting...' here is cruel: the usual reason it never finishes is
        that AMCL took the pose but has no laser scans, so it never runs a filter
        update and never publishes map -> odom. That is invisible from RViz, and
        indistinguishable from having mis-clicked. So report both inputs AMCL
        needs — your click, and the scans — and name the likely fix.
        """
        start = time.time()
        end = start + timeout
        clicks_at_start = self.pose_clicks
        self.scans = 0
        last_report = time.time()

        while time.time() < end and rclpy.ok():
            if self.has_map_frame():
                took = time.time() - start
                print(f"  {OK} AMCL localized after {took:.0f}s.")
                return True
            rclpy.spin_once(self, timeout_sec=0.2)

            if time.time() - last_report < report_every:
                continue

            clicks = self.pose_clicks - clicks_at_start
            scan_hz = self.scans / (time.time() - last_report)
            self.scans = 0
            last_report = time.time()
            waited = int(time.time() - start)

            if clicks == 0:
                print(f"  {DIM}[{waited:3d}s] no pose received yet. In RViz: set Fixed "
                      f"Frame to 'map', click '2D Pose Estimate', then drag an arrow "
                      f"on the map where the robot is.{RESET}")
            elif scan_hz < 0.5:
                # The important one, and the reason this used to hang silently.
                print(f"  {WARN} [{waited:3d}s] AMCL received your pose ({clicks} so far) "
                      f"but there are NO laser scans ({scan_hz:.1f} Hz).")
                print(f"        AMCL only publishes map->odom after a scan update, so "
                      f"it will wait forever like this.")
                print(f"        The lidar is the thing to fix — check it is undocked "
                      f"and try: ros2 service call /start_motor std_srvs/srv/Empty '{{}}'")
            else:
                print(f"  {DIM}[{waited:3d}s] pose received ({clicks}), scans {scan_hz:.1f} Hz "
                      f"— waiting for AMCL to converge...{RESET}")

        print(f"  {WARN} gave up after {timeout:.0f}s.")
        return self.has_map_frame()

    def set_initial_pose(self, x, y, yaw):
        """Tell AMCL where the robot is, instead of clicking in RViz."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance = list(INITIAL_COVARIANCE)
        # AMCL can miss a single message while it is still starting up, so send
        # a few and give it time to run a filter update between them.
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            self.listen(0.4)

    def current_pose(self):
        """Where AMCL currently thinks the robot is, as (x, y, yaw)."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _rgb(self, _m):
        self.rgb += 1

    def _depth(self, _m):
        self.depth += 1

    def _dock(self, m):
        self.docked = m.is_docked

    def _scan(self, _m):
        self.scans += 1

    def call_service(self, name, srv_type, timeout=10.0):
        """Call a service and wait for it. Returns True only on a real response.

        These are ROS *services* (one request, one response), not topics. The
        camera and lidar drivers on this robot advertise them even when they are
        too wedged to answer, so a call that times out is itself a useful signal:
        the driver is beyond recovery and the stack needs restarting.
        """
        client = self.create_client(srv_type, name)
        # Service discovery is subject to the same delay as topic discovery, and
        # on this graph (robot + Nav2 + localization + RViz) that was measured at
        # up to 8 s. A short wait here reports a perfectly healthy service as
        # "did not respond" and sends you off restarting the robot for nothing.
        if not client.wait_for_service(timeout_sec=12.0):
            return False
        future = client.call_async(srv_type.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done() and future.result() is not None

    def cycle_camera(self):
        """Power-cycle the OAK-D via turtlebot4_node's services.

        The camera regularly comes up "ready" — device connected, pipeline
        built, topics advertised — while publishing nothing at all. Stopping and
        restarting it clears that, and is far quicker than restarting the whole
        robot stack.
        """
        print(f"  {DIM}camera is not streaming — trying stop_camera/start_camera...{RESET}")
        if not self.call_service("/oakd/stop_camera", Trigger):
            print(f"  {WARN} stop_camera did not respond.")
            return False
        self.listen(3.0)
        if not self.call_service("/oakd/start_camera", Trigger):
            print(f"  {WARN} start_camera did not respond.")
            return False
        # The pipeline takes a while to rebuild before frames appear.
        return self.wait_for(self.streaming, 30, "the camera to start streaming")

    def start_lidar(self):
        """Ask the lidar driver to spin the motor up.

        Only works if the driver bound to the serial port at startup. If it did
        not, this service is advertised but never answers — which is exactly the
        case where a restart (while undocked) is the only cure.
        """
        print(f"  {DIM}no scans — trying the start_motor service...{RESET}")
        if not self.call_service("/start_motor", Empty):
            print(f"  {WARN} start_motor did not respond — the driver is wedged.")
            return False
        return self.wait_for(self.scanning, 20, "the lidar to start publishing")

    def streaming(self):
        """True if BOTH rgb and depth are arriving — depth is what we actually need."""
        self.rgb = self.depth = 0
        self.listen(3.0)
        return self.rgb > 0 and self.depth > 0

    def _battery(self, m):
        # Published as a 0.0-1.0 fraction, per the BatteryState convention.
        self.battery = m.percentage

    def battery_pct(self):
        return None if self.battery is None else self.battery * 100.0

    def scanning(self):
        """True if laser scans are arriving — AMCL is dead in the water without them.

        Sampled over a few seconds rather than a fraction of one: the lidar runs
        at ~7 Hz but stutters over Wi-Fi, and a brief empty window is not the
        same as a dead lidar.
        """
        self.scans = 0
        self.listen(3.0)
        return self.scans > 0

    def listen(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def undock(self):
        """Drive off the dock and wait for it to finish. Returns True on success."""
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            print(f"  {FAIL} no /undock action server. Is the robot powered and connected?")
            return False

        print(f"  {DIM}undocking — the robot reverses off the dock, stand clear...{RESET}")
        goal = self.undock_client.send_goal_async(Undock.Goal())
        rclpy.spin_until_future_complete(self, goal, timeout_sec=15.0)
        if not goal.done() or not goal.result().accepted:
            print(f"  {FAIL} undock goal rejected.")
            return False

        result = goal.result().get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=45.0)
        if not result.done():
            print(f"  {FAIL} undock timed out.")
            return False

        # The camera needs a moment to start streaming once off the dock.
        self.listen(3.0)
        return True


def ask_target(node):
    """Ask what to search for, and check the detector actually knows that word."""
    print(f"\n{BOLD}What should the robot look for?{RESET}")
    cols = 4
    for i in range(0, len(SUGGESTED), cols):
        print("   " + "".join(f"{c:<16}" for c in SUGGESTED[i:i + cols]))
    print(f"{DIM}   (any of YOLO's 80 COCO classes works, not only these){RESET}")

    try:
        from ultralytics import YOLO
        from object_detector.detect_3d import MODEL
        # Same weights the detector will load, so the class list validated here
        # is the class list that will actually be searched for.
        known = {n.lower() for n in YOLO(MODEL).names.values()}
    except Exception:
        known = None  # cannot validate; trust the user rather than block them

    while True:
        try:
            target = input(f"\n{BOLD}target> {RESET}").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nCancelled.")
            return None
        if not target:
            continue
        if known is None or target in known:
            return target

        similar = [n for n in sorted(known) if target in n or n in target]
        print(f"  {FAIL} '{target}' is not a class this model knows.")
        if similar:
            print(f"        did you mean: {', '.join(similar[:6])}?")


def ask_algorithm():
    """Ask which search strategy to run. Returns its name, or None if cancelled.

    The list comes from search_algorithms.py, so adding a strategy there makes
    it appear here with no change to this function.
    """
    from object_detector import search_algorithms

    options = [search_algorithms.ALGORITHMS[n] for n in search_algorithms.names()]
    if len(options) == 1:
        return options[0].NAME

    print(f"\n{BOLD}Which search algorithm?{RESET}")
    for i, algo in enumerate(options, 1):
        default = "  (default)" if algo.NAME == search_algorithms.DEFAULT else ""
        print(f"   {BOLD}{i}{RESET}. {algo.TITLE}{DIM}{default}{RESET}")
        # Wrap the description by hand rather than pulling in textwrap for one
        # use; these are short enough that two lines is the worst case.
        words, line = algo.DESCRIPTION.split(), ""
        for w in words:
            if len(line) + len(w) + 1 > 66:
                print(f"{DIM}      {line}{RESET}")
                line = w
            else:
                line = f"{line} {w}".strip()
        if line:
            print(f"{DIM}      {line}{RESET}")

    while True:
        try:
            choice = input(f"\n{BOLD}algorithm> {RESET}").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nCancelled.")
            return None
        if not choice:
            return search_algorithms.DEFAULT
        if choice.isdigit() and 1 <= int(choice) <= len(options):
            return options[int(choice) - 1].NAME
        if choice in search_algorithms.ALGORITHMS:
            return choice
        print(f"  {FAIL} pick 1-{len(options)}, or a name "
              f"({', '.join(search_algorithms.names())}).")


# The lifecycle nodes Nav2 brings up, in roughly the order they appear. Used
# only to report progress while waiting, so a slow bringup looks like progress
# rather than a hang.
NAV2_NODES = ["controller_server", "planner_server", "behavior_server",
              "bt_navigator", "smoother_server", "velocity_smoother",
              "collision_monitor", "waypoint_follower"]
LOCALIZATION_NODES = ["map_server", "amcl"]


ROBOT_HOST = os.environ.get("TURTLEBOT4_HOST", "ubuntu@<ROBOT_IP>")
ROBOT_SERVICE = "turtlebot4.service"


def ensure_rviz(node, stack, why="", auto=True):
    """Make sure RViz is open, starting it if it is not. True if it is up.

    Presence is re-checked every time rather than remembered: RViz is a GUI the
    user can close, and it can also die on its own — on 2026-08-18 one was alive
    when the check ran and gone by the time a pose click was needed, so the run
    asked for a click at a window that no longer existed.

    Starting it is also verified. A GUI that exits immediately (the snap library
    clash below is the usual cause) otherwise looks identical to one that
    started fine, because the launch command itself returns success either way.
    """
    if node.settle(node.rviz_up, 3.0):
        return True

    if snap_confined():
        # Starting it from here would just abort on a snap library.
        print(f"  {WARN} RViz is not open, and this terminal is snap-confined")
        print(f"        (VS Code snap), so RViz cannot be started from it.")
        print(f"        Open a NORMAL terminal — not the VS Code one — and run:")
        print(f"        ros2 launch turtlebot4_viz view_navigation.launch.py")
        return False

    if not auto:
        print(f"  {WARN} RViz is not running. Open it in another terminal:")
        print(f"        ros2 launch turtlebot4_viz view_navigation.launch.py")
        return False

    print(f"  {DIM}RViz is not open{f' — {why}' if why else ''}. Starting it.{RESET}")
    proc = stack.start("rviz", [
        "ros2", "launch", "turtlebot4_viz", "view_navigation.launch.py",
    ], keep=True)

    if node.wait_for(node.rviz_up, 45, "RViz to open"):
        return True

    # Distinguish "slow" from "dead": if the process is gone, the log has the
    # reason and waiting longer will never help.
    if proc.poll() is not None:
        print(f"  {FAIL} RViz exited immediately — see logs/mission/rviz.log")
    else:
        print(f"  {WARN} RViz did not appear in the graph within 45s.")
    return False


def restart_robot_service(node):
    """Restart the robot's ROS stack over SSH. Returns True if it ran.

    This is the only cure for the failure that dominated development: docking
    cuts power to the lidar (power_saver), and on undock the driver tries to
    scan before the disc is at speed, fails with 'Cannot start scan', and the
    process EXITS. Nothing restarts it, so /scan stays dead forever and
    start_motor cannot help — the process owning that service is gone.

    Two hard rules:
      * Never restart while docked. The driver would come up against an
        unpowered lidar and die exactly the same way.
      * Never hardcode the robot's password. It is read from the environment,
        or better, passwordless sudo is configured for this one command.
    """
    if node.docked:
        print(f"  {FAIL} refusing to restart while DOCKED — the driver would start")
        print(f"        against an unpowered lidar and die the same way. Undock first.")
        return False

    print(f"\n  {BOLD}The lidar driver has exited; only a restart of the robot's")
    print(f"  ROS stack can bring it back.{RESET}")
    print(f"  {DIM}This stops and restarts every driver on the robot (~50 s).{RESET}")
    if input("  Restart the robot service now? [y/N] ").strip().lower() not in ("y", "yes"):
        print(f"  {DIM}skipped.{RESET}")
        return False

    base = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", ROBOT_HOST]
    password = os.environ.get("TURTLEBOT4_PASSWORD")

    # Prefer passwordless sudo; fall back to a password from the environment.
    attempts = [("passwordless sudo", base + [f"sudo -n systemctl restart {ROBOT_SERVICE}"])]
    if password:
        attempts.append(("TURTLEBOT4_PASSWORD", base + [
            f"echo '{password}' | sudo -S systemctl restart {ROBOT_SERVICE}"]))

    for label, cmd in attempts:
        print(f"  {DIM}restarting via {label}...{RESET}")
        if subprocess.run(cmd, capture_output=True, text=True, timeout=60).returncode == 0:
            print(f"  {OK} restart issued. Waiting for the drivers to come back...")
            return True

    print(f"  {FAIL} could not restart the service automatically.")
    print(f"        Run it by hand:")
    print(f"          ssh {ROBOT_HOST} \\")
    print(f"            \"sudo systemctl restart {ROBOT_SERVICE}\"")
    print(f"        To let this happen automatically in future, allow just this one")
    print(f"        command without a password — on the robot, run:")
    print(f"          echo 'ubuntu ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart "
          f"{ROBOT_SERVICE}' \\")
    print(f"            | sudo tee /etc/sudoers.d/turtlebot4-restart")
    print(f"        (Or export TURTLEBOT4_PASSWORD, though a sudoers rule is safer")
    print(f"        than a password in your environment.)")
    return False


def restart_hint(reason):
    """Tell the operator how to restart the robot stack, with the one caveat."""
    print(f"  {FAIL} {reason}")
    print(f"        Restart the robot service — but ONLY while UNDOCKED. If the")
    print(f"        lidar is unpowered when the driver starts, it fails with")
    print(f"        'Cannot start scan' and the process dies for good:")
    print(f"            ssh ubuntu@<ROBOT_IP> \\")
    print(f"              \"echo $TURTLEBOT4_PASSWORD | sudo -S systemctl restart {ROBOT_SERVICE}\"")
    print(f"        Give it ~50 s, then run this again.")
    print(f"        If that does not help, power-cycle the robot: hold the Create 3")
    print(f"        button until the light ring goes out. A Pi reboot is not enough —")
    print(f"        it does not reset the USB hub the sensors hang off.")


def ensure_undocked(node):
    """Get the robot off the dock before anything else needs its sensors.

    This has to happen first, not as a later check. While docked the RPLiDAR
    stops spinning, so /scan is silent — and AMCL will happily accept an initial
    pose and then never publish map -> odom, because it needs a laser update to
    run the filter. The symptom is that the pose click in RViz appears to do
    nothing at all. The OAK-D is likewise dark on the dock.
    """
    print(f"\n{BOLD}Robot{RESET}")
    node.settle(lambda: node.docked is not None, 12.0)

    # Battery is informational, so never block on it — but give it long enough
    # to be meaningful, since the topic is slow.
    node.settle(lambda: node.battery is not None, 6.0)
    pct = node.battery_pct()
    if pct is None:
        battery_txt = "battery unknown"
        # Say so out loud. Printing nothing here reads as "the feature is
        # broken" rather than "the reading did not arrive".
        print(f"  {WARN} {battery_txt} — no /battery_state received.")
    else:
        battery_txt = f"battery at {pct:.0f}%"
        mark = OK if pct >= 40 else WARN
        print(f"  {mark} {battery_txt}.")
        if pct < 20:
            # A sagging battery is the leading suspect for the OAK-D dropping
            # off mid-session: the camera draws real current once undocked.
            print(f"        Low. A search takes several minutes of driving, and a")
            print(f"        weak battery can brown out the camera. Consider charging.")

    if node.docked is None:
        print(f"  {WARN} could not read /dock_status — is the robot connected?")
        return True
    if not node.docked:
        print(f"  {OK} robot is off the dock.")
    else:
        print(f"  {WARN} robot is DOCKED. While docked the lidar does not spin and")
        print(f"        the camera does not stream, so neither localization nor")
        print(f"        detection can work.")
        if input(f"        {battery_txt} — undock now? [Y/n] "
                 ).strip().lower() not in ("", "y", "yes"):
            print(f"  {FAIL} cannot continue while docked.")
            return False
        if not node.undock():
            return False
        print(f"  {OK} undocked.")

    # Confirm the lidar actually came back, since localization depends on it.
    if node.wait_for(node.scanning, 20, "the lidar to start publishing"):
        print(f"  {OK} lidar publishing.")
    elif node.start_lidar():
        print(f"  {OK} lidar publishing after start_motor.")
    else:
        # Not fatal here — run_checks will try again and give the full advice.
        print(f"  {WARN} still no /scan; will retry in preflight.")
    return True


def bring_up_stack(node, stack, auto):
    """Make sure localization and Nav2 are running, starting them if needed."""
    print(f"\n{BOLD}Navigation stack{RESET}")

    # --- localization -------------------------------------------------------
    if node.settle(node.localization_up, 5.0):
        print(f"  {OK} localization already running (left alone).")
    elif not auto:
        print(f"  {WARN} localization not running, and auto-start declined.")
    elif not os.path.exists(DEFAULT_MAP):
        print(f"  {FAIL} map not found: {DEFAULT_MAP}")
    else:
        stack.start("localization", [
            "ros2", "launch", "turtlebot4_navigation", "localization.launch.py",
            f"map:={DEFAULT_MAP}",
        ])
        if node.wait_for(node.localization_active, 60,
                         "map_server + amcl to activate",
                         progress=node.localization_states):
            print(f"  {OK} localization active.")
        else:
            # The nodes are almost certainly running — the manager just failed
            # to finish driving them. Finish the job rather than giving up.
            print(f"  {WARN} lifecycle bringup stalled ({node.localization_states()}).")
            print(f"  {DIM}driving the transitions by hand...{RESET}")
            if node.force_localization_active():
                print(f"  {OK} localization active after manual transition.")
            else:
                print(f"  {FAIL} localization did not come up — "
                      f"see logs/mission/localization.log")

    # --- initial pose -------------------------------------------------------
    # This is the only step that genuinely needs a human, because AMCL cannot
    # know where the robot is. A recorded start pose stands in for the click.
    if node.has_map_frame():
        print(f"  {OK} 'map' frame is up — robot is localized.")
    elif node.localization_up():
        saved = load_start_pose()
        use_saved = False
        if saved:
            x, y, yaw = saved["x"], saved["y"], saved["yaw"]
            was_docked, when = saved["docked"], saved["recorded"]
            where = {True: "on the dock", False: "off the dock",
                     None: "dock state not recorded"}[was_docked]
            print(f"  saved start pose: ({x:+.2f}, {y:+.2f}, {math.degrees(yaw):.0f}°)")
            print(f"        recorded {when}, {where}")

            # A pose taken in the other dock state is ~0.3 m out, which AMCL may
            # or may not recover from. Say so plainly rather than seeding it and
            # hoping.
            mismatch = was_docked is not None and node.docked is not None \
                and was_docked != node.docked
            if mismatch:
                print(f"  {WARN} the robot is currently "
                      f"{'ON' if node.docked else 'OFF'} the dock, but this pose was")
                print(f"        recorded {where} — undocking moves it about 0.3 m, so")
                print(f"        this pose is likely wrong. Setting it by hand is safer.")
            elif was_docked is None:
                print(f"  {WARN} this pose predates dock-state recording, so it cannot")
                print(f"        be checked against where the robot is now.")

            # The saved pose is only right if the robot is actually back in that
            # spot. It usually is, so default to yes — but never assume it.
            default_yes = not mismatch
            prompt = "[Y/n]" if default_yes else "[y/N]"
            answer = input(f"  Use this saved pose? (n = set it yourself in RViz) "
                           f"{prompt} ").strip().lower()
            use_saved = (answer in ("", "y", "yes")) if default_yes else (answer in ("y", "yes"))

        localized = False
        if use_saved:
            print(f"  {DIM}setting initial pose from "
                  f"{os.path.basename(START_POSE_FILE)}...{RESET}")
            node.set_initial_pose(x, y, yaw)
            if node.wait_for_pose(30, report_every=10.0):
                print(f"  {OK} localized automatically — no RViz click needed.")
                print(f"  {DIM}Check in RViz that the laser scan lines up with the "
                      f"walls. If not, re-run and answer 'n' here.{RESET}")
                localized = True
            else:
                print(f"  {WARN} AMCL did not converge from the saved pose.")

        # Fall through to a manual pose whether the saved one was declined or
        # tried and failed. Previously a failed saved pose only printed advice
        # and carried on, so the run continued unlocalized and died at preflight
        # complaining about a missing 'map' frame several steps later.
        if not localized:
            if not saved:
                print(f"  {WARN} no saved start pose, so AMCL does not know "
                      f"where the robot is.")
            # Asking for a click in RViz is useless if RViz is not running.
            ensure_rviz(node, stack, why="you need to set the robot's pose",
                        auto=auto)
            print(f"        Set Fixed Frame to 'map', then click '2D Pose Estimate'")
            print(f"        and drag an arrow where the robot actually is.")
            if node.wait_for_pose(300):
                pose = node.current_pose()
                if pose and input(f"  save this as the start pose for next time? [Y/n] "
                                  ).strip().lower() in ("", "y", "yes"):
                    # Record the dock state too, so a later run can tell whether
                    # the robot is back in the same place.
                    save_start_pose(*pose, docked=node.docked)
                    print(f"  {OK} saved to {START_POSE_FILE} — future runs will "
                          f"localize themselves.")

    # RViz is also how you watch the search and see the object map markers, so
    # bring it up even when no pose click was needed — otherwise a run that
    # localizes from the saved pose never opens a window at all.
    if auto:
        ensure_rviz(node, stack, why="so you can watch the search", auto=auto)

    # --- nav2 ---------------------------------------------------------------
    if node.nav2_up(timeout=2.0):
        print(f"  {OK} Nav2 already running (left alone).")
    elif not auto:
        print(f"  {WARN} Nav2 not running, and auto-start declined.")
    else:
        params = DEFAULT_NAV2_PARAMS if os.path.exists(DEFAULT_NAV2_PARAMS) else None
        cmd = ["ros2", "launch", "turtlebot4_navigation", "nav2.launch.py"]
        if params:
            cmd.append(f"params_file:={params}")
        stack.start("nav2", cmd)
        if not node.wait_for(lambda: node.nav2_up(0.5), 60, "Nav2 to accept goals",
                             progress=lambda: node.nodes_present(NAV2_NODES)):
            print(f"  {FAIL} Nav2 did not come up — see logs/mission/nav2.log")

    for name, proc, path in stack.died():
        print(f"  {FAIL} {name} exited immediately (code {proc.returncode}) — see {path}")


def run_checks(node, target):
    """Run every precondition. Returns (blocking_failures, warnings)."""
    print(f"\n{BOLD}Preflight{RESET}")
    blocking, warnings = [], []

    # --- dock ---------------------------------------------------------------
    # Handled up front by ensure_undocked(); this only confirms nothing changed.
    node.listen(1.5)
    if node.docked:
        print(f"  {FAIL} robot is back on the dock — lidar and camera are off.")
        blocking.append("robot is docked")
    else:
        print(f"  {OK} robot is off the dock.")

    # --- lidar --------------------------------------------------------------
    # settle() retries the check for up to 20 s. A single empty sample is not
    # evidence of a dead lidar — it is often just this process being starved.
    if node.settle(node.scanning, 20.0):
        print(f"  {OK} lidar publishing.")
    elif node.start_lidar():
        print(f"  {OK} lidar publishing after start_motor.")
    elif restart_robot_service(node) and node.wait_for(node.scanning, 90,
                                                       "the lidar after the restart"):
        print(f"  {OK} lidar publishing after restarting the robot service.")
    else:
        restart_hint("no /scan, and neither start_motor nor a service restart "
                     "recovered it. AMCL cannot localize without scans.")
        blocking.append("lidar not publishing")

    # --- camera -------------------------------------------------------------
    if node.settle(node.streaming, 20.0):
        print(f"  {OK} camera streaming (rgb {node.rgb}, depth {node.depth}).")
    else:
        # Depth is the half that matters: without it a detection has no range and
        # cannot be placed on the map. Missing either half gets the same cure, so
        # do not bother distinguishing them before trying it.
        if node.rgb and not node.depth:
            print(f"  {WARN} RGB arriving but NO depth — no depth means no distance.")
        elif node.depth and not node.rgb:
            print(f"  {WARN} depth arriving but no RGB.")
        else:
            print(f"  {WARN} no camera data at all in 4 s.")

        if node.cycle_camera():
            print(f"  {OK} camera streaming after a stop/start cycle.")
        elif "lidar not publishing" not in blocking and restart_robot_service(node) \
                and node.wait_for(node.streaming, 90, "the camera after the restart"):
            # Skipped if the lidar check already restarted the stack — one
            # restart fixes both, and asking twice in a row is just noise.
            print(f"  {OK} camera streaming after restarting the robot service.")
        else:
            restart_hint("the camera is still not streaming after a stop/start cycle.")
            blocking.append("camera not streaming")

    # --- map frame ----------------------------------------------------------
    # settle() for the same reason as the checks above: a tf buffer on a
    # freshly created node needs a couple of seconds to fill (measured 2.1 s
    # against a healthy, localized stack), so a single sample here reports
    # "no map frame" on a system that is working perfectly well.
    if node.settle(node.has_map_frame, 20.0):
        print(f"  {OK} 'map' frame is up — results will be in map coordinates.")
    else:
        print(f"  {FAIL} no 'map' frame. Start localization and set the robot's pose:")
        print(f"        ros2 launch turtlebot4_navigation localization.launch.py "
              f"map:=$HOME/turtlebot4_ws/maps/room_a.yaml")
        print(f"        then click '2D Pose Estimate' in RViz.")
        blocking.append("no map frame")

    # --- nav2 ---------------------------------------------------------------
    if node.nav_client.wait_for_server(timeout_sec=5.0):
        print(f"  {OK} Nav2 is accepting goals.")
    else:
        print(f"  {FAIL} Nav2 is not running — the robot cannot drive to waypoints.")
        print(f"        ros2 launch turtlebot4_navigation nav2.launch.py "
              f"params_file:=$HOME/turtlebot4_ws/config/nav2_lab.yaml")
        blocking.append("Nav2 not running")

    # --- waypoints ----------------------------------------------------------
    wp_file = os.environ.get("SEARCH_WAYPOINTS", DEFAULT_WAYPOINTS)
    try:
        with open(wp_file) as f:
            wps = yaml.safe_load(f).get("waypoints", [])
        if wps:
            print(f"  {OK} {len(wps)} waypoints loaded from {os.path.basename(wp_file)}.")
        else:
            print(f"  {FAIL} {wp_file} has no waypoints.")
            blocking.append("no waypoints")
    except Exception as e:
        print(f"  {FAIL} could not read {wp_file}: {e}")
        blocking.append("waypoints unreadable")

    return blocking, warnings, wp_file


def main(args=None):
    print(f"\n{BOLD}=== TurtleBot 4 object search ==={RESET}")

    rclpy.init(args=args)
    node = Preflight()
    stack = StackManager()
    search_cmd = None

    try:
        target = ask_target(node)
        if target is None:
            return

        # Let DDS discovery finish before deciding what is running. Probing
        # immediately reports an empty graph, and acting on that would start a
        # second map_server and amcl alongside the ones already up.
        if not ensure_undocked(node):
            return

        need = []
        if not node.settle(node.localization_up, 8.0):
            need.append("localization")
        if not node.nav2_up(timeout=2.0):
            need.append("Nav2")
        auto = True
        if need:
            print(f"\n{' and '.join(need)} not running.")
            auto = input(f"Start {'them' if len(need) > 1 else 'it'} in the "
                         f"background? [Y/n] ").strip().lower() in ("", "y", "yes")

        bring_up_stack(node, stack, auto)

        # Start preflight from a brand-new node when we just launched the stack.
        #
        # Bringing up Nav2 and localization adds ~15 participants to the graph at
        # once, and this process's existing participant comes out of that storm
        # deaf: every subscription (scan, camera, tf) reports nothing while the
        # data is demonstrably still flowing — a fresh node created seconds later
        # sees all of it. Without this, preflight blames the robot for what is
        # really our own stale participant, and sends you off restarting hardware
        # that was never broken.
        if stack.started:
            print(f"\n{DIM}letting the stack settle before preflight...{RESET}")
            node.destroy_node()
            node = Preflight()
            node.settle(lambda: node.docked is not None, 10.0)
            # Nav2's startup saturates this laptop's CPU for a while (its own
            # bringup needs bond_timeout raised to 20 s for the same reason).
            # Judging the sensors during that window measures our own starved
            # callbacks, not the robot: preflight would report no scans and no
            # camera while both were demonstrably streaming.
            node.listen(12.0)

        blocking, warnings, wp_file = run_checks(node, target)
        if blocking:
            print(f"\n{BOLD}Not ready.{RESET} Fix these first:")
            for b in blocking:
                print(f"  - {b}")
            print("\nThen run this again.")
            # Tearing the stack down here means the next attempt has to bring
            # localization up and localize all over again, which is slow and
            # loses the 'map' frame in RViz. When the run failed on a sensor,
            # that stack is still perfectly good — offer to leave it up.
            if stack.started:
                keep = input("\nKeep localization/Nav2 running for the next "
                             "attempt? [Y/n] ").strip().lower() in ("", "y", "yes")
                if keep:
                    stack.started = [(n_, p_, path_, True)
                                     for n_, p_, path_, _ in stack.started]
                    print(f"  {OK} left running — RViz keeps its 'map' frame, and "
                          f"the next run will reuse them.")
            return

        algorithm = ask_algorithm()
        if algorithm is None:
            return

        print(f"\n{BOLD}Ready to search for '{target}' using {algorithm}.{RESET}")
        print(f"{DIM}The robot will drive itself around the room. Clear the floor.{RESET}")
        if input("Start? [Y/n] ").strip().lower() not in ("", "y", "yes"):
            print("Cancelled.")
            return

        search_cmd = [
            "ros2", "launch", "object_detector", "search.launch.py",
            f"target_class:={target}", f"waypoints_file:={wp_file}",
            f"algorithm:={algorithm}",
        ]
    except (EOFError, KeyboardInterrupt, ExternalShutdownException):
        print("\nCancelled.")
        return
    finally:
        node.destroy_node()
        # Release DDS before handing the terminal to the launch below, so the
        # two do not fight over discovery.
        if rclpy.ok():
            rclpy.shutdown()
        if search_cmd is None:
            stack.stop_all()

    print(f"\n{DIM}$ {' '.join(search_cmd)}{RESET}\n")
    code = 0
    try:
        code = subprocess.call(search_cmd)
    except KeyboardInterrupt:
        print("\nSearch interrupted.")
    finally:
        if stack.started:
            print(f"\n{BOLD}Shutting down the stack this run started.{RESET}")
            stack.stop_all()
    sys.exit(code)


if __name__ == "__main__":
    main()
