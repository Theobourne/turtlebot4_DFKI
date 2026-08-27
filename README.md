# TurtleBot 4 — Semantic Object Search

Given the name of an object, this system drives a TurtleBot 4 around a mapped
room, looks for the object with a YOLO detector, and reports **where the object
is in map coordinates**.

It is the sensing and navigation substrate for research into *search stopping
policies* — deciding when a robot should give up on a room and move on. Every
component here is deliberately off-the-shelf; the research contribution sits on
top of it, not inside it.

---

## What it does

```
  target: "chair"
        │
        ▼
  Nav2 waypoint tour ──► stop, sweep 360° ──► YOLO detection
        │                                            │
        │                                     bbox centre pixel
        │                                            ▼
        │                                    depth image lookup
        │                                            ▼
        │                            unproject through camera intrinsics
        │                                            ▼
        │                          tf2: camera optical frame ──► map
        ▼                                            │
  next waypoint ◄──── not enough sightings ──────────┤
                                                     ▼
                                    cluster sightings ──► map coordinates
```

A detection becomes a map coordinate in four steps: read the depth at the
bounding-box centre, back-project that pixel through the camera intrinsics into
a 3-D point, transform it into the map frame with tf2, then cluster repeated
sightings so the answer rests on several looks rather than one lucky frame.

---

## Quick start

One command runs everything, including bringing up localization and Nav2 if they
are not already running:

```bash
cd ~/turtlebot4_ws
source install/setup.bash
ros2 run object_detector mission
```

It asks what to search for, then checks the robot is actually ready — undocking
it if needed — before anything moves. Each failed check prints the command that
fixes it.

```
=== TurtleBot 4 object search ===

What should the robot look for?
   chair           person          bottle          laptop
   ...
target> chair

Navigation stack
    ok    localization already running (left alone).
    ok    'map' frame is up — robot is localized.
    ok    Nav2 already running (left alone).

Preflight
    ok    robot is off the dock.
    ok    camera streaming (rgb 31, depth 9 in 4 s).
    ok    5 waypoints loaded from search_waypoints.yaml.

Ready to search for 'chair'.
```

### Running the pieces separately

```bash
# Detection with map-frame localization, no navigation
ros2 run object_detector detect_3d

# Restrict to one class
ros2 run object_detector detect_3d --ros-args -p target_class:=chair

# Report relative to the robot instead of the map (no Nav2 needed)
ros2 run object_detector detect_3d --ros-args -p target_frame:=base_link

# Detection only, no depth or localization (the original node)
ros2 run object_detector detect
```

`detect_3d` falls back to `base_link` automatically when there is no `map`
frame, and switches to `map` on its own once AMCL is localized — no restart
needed. Every log line names the frame it used:

```
[map] chair 0.85 @ (+2.75, +0.19) 2.81 m out
```

---

## Packages and nodes

| Node | Package | Purpose |
|---|---|---|
| `mission` | `object_detector` | Interactive entry point: target selection, stack bring-up, preflight checks |
| `search` | `object_detector` | Nav2 waypoint tour; clusters sightings and reports the final coordinate |
| `detect_3d` | `object_detector` | YOLO + depth → map-frame points, markers, annotated image |
| `detect` | `object_detector` | Original detection-only node (no depth, no localization) |
| `grab_frames` | `frame_grabber` | 90-slot ring buffer of camera frames for offline work |

### Topics published

| Topic | Type | Description |
|---|---|---|
| `/detections/object_point` | `geometry_msgs/PointStamped` | One per accepted detection, in the target frame |
| `/detections/markers` | `visualization_msgs/MarkerArray` | Detection spheres for RViz |
| `/detections/annotated/compressed` | `sensor_msgs/CompressedImage` | Boxes with range labels |
| `/search/result_marker` | `visualization_msgs/MarkerArray` | Final located object, with label |

---

## Configuration

| File | Purpose |
|---|---|
| `config/search_waypoints.yaml` | Map-frame waypoints for the search tour |
| `config/start_pose.yaml` | Recorded start pose, so AMCL localizes without an RViz click (created on first run) |
| `config/nav2_lab.yaml` | Nav2 parameters tuned for this lab's Wi-Fi — see the setup log |
| `maps/room_a.yaml` / `.pgm` | SLAM map of the search room |

### Search algorithms

The strategy — *where to look next* — is separated from the machinery that
collects sightings, clusters them, saves proof and docks. Pick one at the
`mission` prompt, or pass it directly:

```bash
ros2 launch object_detector search.launch.py target_class:=chair algorithm:=nearest-first
```

| Name | Module | Strategy |
|---|---|---|
| `kmeans-tour` *(default)* | `kmeans_waypoint_tour.py` | Visits the k-means coverage waypoints in file order. Deterministic baseline. |
| `nearest-first` | `nearest_waypoint_tour.py` | Same waypoints and sweep, but always drives to the closest unvisited one. Same coverage, less travel. |

### Continuous sensing

By default the robot keeps looking **while it drives**, instead of only at
waypoints. Detections gathered in motion are treated as *cues*, not evidence —
motion blur, noisier stereo and tf error that grows with speed make them good
enough to say "something over there" and not good enough to answer with.

A cue interrupts the drive. The robot stops, faces the candidate and looks from
a standstill, then confirms only if agreeing sightings came from two viewpoints
far enough apart to be independent looks rather than one look sampled twice.

The parallax often comes free: if the approach passed the candidate obliquely,
its cue sightings already span a usable baseline and no extra move is needed.
Otherwise the robot sidesteps. The offset scales with range to hold ~12° of
parallax (0.40 m floor, 1.00 m ceiling) and the **achieved** baseline is
measured from tf, never assumed — Nav2's `xy_goal_tolerance` is 0.25 m, more
than half the smallest sidestep ever requested.

Failed candidates are blacklisted for the rest of the run, and verifications are
capped at 3 so a flapping detector cannot consume the battery.

```bash
# turn it off — recommended when comparing search strategies against each other
ros2 launch object_detector search.launch.py target_class:=chair continuous_sensing:=false
```

Leave it **off for strategy comparisons**: finding the target between waypoints
makes every strategy look alike, which is exactly the difference an A/B is
trying to measure. Both the startup log and the final summary state which mode
ran.

To add a strategy, write a module exposing `NAME`, `TITLE`, `DESCRIPTION` and
`run(nav, collector, waypoints, target)`, then list it in
`search_algorithms.py`. The menu, the `algorithm` parameter and the launch file
all read that registry, so nothing else needs changing. Every run reports its
algorithm and elapsed search time, so strategies can be compared directly.

Tuning knobs shared by all of them are constants at the top of
`src/object_detector/object_detector/search_common.py`:

| Constant | Default | Effect |
|---|---|---|
| `DWELL_SECONDS` | `3.0` | How long to stand and look at each sweep step |
| `SWEEP_STEPS` | `4` | In-place turns per waypoint (4 = full circle) |
| `CLUSTER_RADIUS` | `0.6` | Sightings closer than this are treated as one object (m) |
| `MIN_SIGHTINGS` | `3` | Agreeing looks required before declaring the object found |

The waypoints shipped in `search_waypoints.yaml` were derived from the map
itself rather than guessed: the free space of `room_a.pgm` was distance-
transformed to keep 0.45 m of wall clearance, then k-means split the reachable
44.6 m² into five regions. **Check them in RViz before a run** — furniture
moves, and they are only as good as the map.

---

## Setup notes specific to this robot

Two findings cost significant debugging time and are worth knowing before
touching the camera code.

**The camera does not stream while docked.** The OAK-D publishes nothing on the
dock, so detection appears silently broken. `mission` reads `/dock_status` and
offers to undock; `detect_3d` says so in its heartbeat.

**Use the compressed image topics, with RELIABLE QoS.** The raw topics
(`/oakd/rgb/image_raw`, `/oakd/stereo/image_raw`) deliver **0 Hz** to the
workstation over lab Wi-Fi — at 2.7 MB and 1.8 MB per frame they are fragmented
across many UDP packets and effectively never arrive intact, while the same
topics run at 4 and 10 Hz measured on the Pi itself. The compressed pair
delivers ~2 Hz of time-synchronised frames:

- `/oakd/rgb/image_raw/compressed` (~177 KB JPEG)
- `/oakd/stereo/image_raw/compressedDepth` (~355 KB PNG)

`compressedDepth` is lossless PNG behind a short binary header, so it decodes to
exactly the uint16 1280×720 millimetre image the raw topic would have carried.
Same resolution means the RGB intrinsics apply unchanged and a bounding-box
pixel indexes straight into it.

Critically, these topics are published **RELIABLE**, and subscribers must match.
`qos_profile_sensor_data` (BEST_EFFORT) yields **zero** frames here: a
multi-fragment sample that loses one fragment is discarded with no retransmit.
This is the opposite of the usual advice for sensor topics.

---

## Troubleshooting

`detect_3d` prints a status line every 5 seconds, so a stalled pipeline always
says which stage is at fault rather than going quiet.

| Message | Meaning |
|---|---|
| `no camera data at all — IS THE ROBOT DOCKED?` | Nothing ever arrived. Undock. |
| `camera stream STALLED — it was working` | Was streaming, now not. Wi-Fi or the `/oakd` node died. |
| `RGB arriving but NO depth` | Camera is not in RGBD pipeline mode |
| `both streams arriving but NOTHING pairs up` | Timestamps disagree — clock skew, or too sparse to pair |
| `YOLO saw nothing above conf 0.4` | Pipeline healthy, nothing in view |
| `status: 10 pairs (2.0 Hz), 3 objects, 2 published` | Healthy |

If the robot will not move, or Nav2 will not come up, see
[`turtlebot4_setup_log.md`](turtlebot4_setup_log.md) — it documents the clock
synchronisation stack, Nav2 bring-up ordering, and the `cmd_vel` chain.

---

## Building

```bash
cd ~/turtlebot4_ws
colcon build --packages-select object_detector --symlink-install
source install/setup.bash
```

Requires ROS 2 Jazzy, Nav2, and `ultralytics`. The YOLO weights (`yolov8n.pt`)
are not tracked in this repository; Ultralytics downloads them on first use.

---

## Repository layout

```
config/          Nav2 params, search waypoints, recorded start pose
maps/            SLAM maps (room_a)
src/
  object_detector/   detection, localization, search, mission entry point
  frame_grabber/     camera frame ring buffer
turtlebot4_setup_log.md   robot and workstation configuration history
```

---

## Status

Working and verified on hardware:

- Detection with map-frame localization, validated against ground truth
  (a chair 2.61 m down the optical axis resolves to x = +2.55 m in `base_link`,
  matching the camera's 0.060 m forward offset)
- Sighting clustering
- Preflight checks and background stack bring-up
- Automatic initial-pose replay from a recorded start pose

Not yet exercised on hardware:

- The full Nav2 waypoint drive — `goToPose`, the per-waypoint sweep, and the
  found/not-found report. Expect to tune the search constants above on the first
  real run.
