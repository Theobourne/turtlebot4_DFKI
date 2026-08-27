# TurtleBot 4 — Setup & System Change Log

Running record of configuration and system-level changes made to the robot and
the workstation during the object-search project. Kept for reproducibility and
for handover to the colleague inheriting the robot.

**Append new entries at the bottom, newest last. Date every entry.**

---

## Quick reference

| Thing | Value |
|---|---|
| Robot (Pi) SSH | `ubuntu@<ROBOT_IP>` — password not recorded here; see your own credential store |
| Robot hostname | `turtlebot4` |
| Workstation | `<user>@<workstation>` (`<WORKSTATION_IP>`), Ubuntu 24.04 |
| ROS distro | Jazzy (both machines) |
| RMW / domain | `rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0` |
| Map | `~/turtlebot4_ws/maps/room_a.yaml` / `.pgm` (337×340 @ 0.05 m/cell) |
| Nav2 params (tuned) | `~/turtlebot4_ws/config/nav2_lab.yaml` (+ `.working` backup) |
| Create 3 base (internal) | `<CREATE3_USB_IP>`, Pi side is `<PI_USB_IP>` |
| Trial bags land in | `~/trials/` on the Pi (MCAP) |
| Recording script | `~/record_trial.sh` on the Pi |
| Camera RGB (use this) | `/oakd/rgb/image_raw/compressed`, 1280×720, **RELIABLE** |
| Camera depth (use this) | `/oakd/stereo/image_raw/compressedDepth`, 16UC1 mm, aligned to RGB |
| Camera intrinsics | `fx 1027.002, fy 1026.493, cx 635.739, cy 373.907` |
| ⚠️ Raw image topics | `image_raw` without `/compressed` delivers **0 Hz** off-robot — see 2026-08-12 |

---

## Startup sequence (follow this order — see 2026-08-10 entries for why)

1. **Check clocks.** On the laptop: `chronyc tracking | grep 'Ref time'` vs
   `date -u`. Then `ssh ubuntu@<ROBOT_IP> 'date +%s'; date +%s`.
2. **Robot bringup.** Verify `/scan` and `/odom` are both flowing at sane rates
   (`ros2 topic hz`), and that each stamp is within ~1 s of wall clock.
3. **Localization.** Full path, no `~`:
   ```
   ros2 launch turtlebot4_navigation localization.launch.py \
     map:=$HOME/turtlebot4_ws/maps/room_a.yaml
   ```
4. **RViz.** Set Fixed Frame to `map`. Set 2D Pose Estimate.
   Verify with `ros2 run tf2_ros tf2_echo map odom` — must resolve.
5. **Nav2 — only after step 4 succeeds.**
   ```
   ros2 launch turtlebot4_navigation nav2.launch.py \
     params_file:=$HOME/turtlebot4_ws/config/nav2_lab.yaml
   ```
6. **Verify the whole stack** with `navcheck`. Every node must read
   `active [3]`. If any are not, **relaunch** — do not hand-activate.

---

## Change log

### 2026-07-14 — Data logging pipeline established
- Created `~/record_trial.sh` on the Pi. Records a curated topic set to an
  **MCAP** bag under `~/trials/<stamp>_<room>_<method>_t<trial>/`.
- Usage: `./record_trial.sh <room> <method> [trial_no]`, `Ctrl+C` to stop.
- Verified end-to-end: state, odom, `/scan`, `/tf` all capturing; camera
  confirmed at ~25 frames/s into the bag.
- Rationale: single replayable artifact per trial for analysis and handover.

### 2026-07-14 — Camera topic name corrected in script
- The OAK-D on this robot publishes under `/oakd/rgb/preview/...`, **not**
  `/oakd/rgb/...`. Original script pointed at the wrong namespace and silently
  recorded no camera data.
- Updated `CAM_TOPICS` in `record_trial.sh` to:
  - `/oakd/rgb/preview/image_raw/compressed`
  - `/oakd/rgb/preview/camera_info`
- ⚠️ Note: `preview` is the low-res (~250×250) stream. Fine for pipeline
  testing; not adequate for detection work. Camera has since been reconfigured
  to 720p (see below).

### 2026-07-14 — System clock set manually on the Pi
- Symptom: bags were timestamped **Jun 5 2026** while the real date was
  Jul 14 — the Pi's clock was ~5 weeks stale, NTP not syncing.
- Action taken: `sudo timedatectl set-ntp false`, then
  `sudo timedatectl set-time "<current local time>"`.
- ⚠️ **Superseded.** Manual time drifts. Replaced by the chrony setup described
  in the 2026-08-10 entries. Do not set time manually any more.

### 2026-08-10 — Clock stack fully diagnosed (THREE clocks, three failure modes)

The robot has **three independent clocks**: the laptop, the Pi, and the Create 3
base. All three must agree or tf silently drops sensor data and Nav2 fails in
confusing, non-obvious ways. The lab firewall blocks outbound UDP/123, so no
machine can reach an external NTP server — the laptop runs on **local stratum**
(`127.127.1.1` / RefID `7F7F0101`) and everything else chains off it:

```
laptop chronyd (local stratum 10)
  └── Pi chronyd  (server <WORKSTATION_IP>)
        └── Create 3 base ntpd  (server <PI_USB_IP>, over internal USB subnet)
```

**Failure mode A — Pi clock drifted alone.**
Fix: `fixbotclock` (bash function in `~/.bashrc`; runs `chronyc makestep` on the
Pi over SSH).

**Failure mode B — the laptop's chronyd drifted its *served* time away from the
laptop's own system clock.** The Pi then faithfully copies a wrong upstream, and
every Pi-side `makestep` reverts within seconds.
- **Diagnostic (SSH-overhead-free):** on the laptop compare
  `chronyc tracking | grep 'Ref time'` against `date -u`. If Ref time is seconds
  off, the *laptop* is the broken clock. Do **not** trust raw `date` comparisons
  over SSH — round-trip overhead repeatedly pointed at the wrong machine.
- **Fix:** `sudo systemctl restart chrony` **on the laptop**, then `fixbotclock`.

**Failure mode C (new, found today) — the Pi's `chrony.conf` had NO `makestep`
line at all.** chronyd could therefore only *slew*, never step. A 92-second
error would have taken roughly 18 minutes to correct, silently.
- **Symptom signature:** scan timestamps sit at a **constant** offset from wall
  clock that does not shrink, while `chronyc sources` shows a healthy `^*` lock
  and `chronyc tracking` reports e.g. `System time: 92.19 seconds slow`.
  Sync is working; the correction just never lands.
- **Fix:** add to `/etc/chrony/chrony.conf` on the Pi, after the `server` line:
  ```
  makestep 1 -1
  ```
  The `-1` means step at *any* update, forever — not just the first three.
  Then `sudo systemctl restart chrony`.
- The Pi's config also carried a duplicate `local stratum 10` line (harmless).

**Create 3 base clock.**
- Its `ntp.conf` is edited through the **Create 3 web server**
  (Beta Features → Edit ntp.conf, then Restart ntpd), **not** over SSH.
- Correct config, already in place — **leave it alone**:
  ```
  server <PI_USB_IP> prefer iburst minpoll 4 maxpoll 6
  ```
- `<CREATE3_USB_IP>` is on the internal Pi↔base USB subnet and is **not reachable
  from the laptop**. Tunnel to it:
  ```
  ssh -L 8080:<CREATE3_USB_IP>:80 ubuntu@<ROBOT_IP>
  ```
  then browse to `localhost:8080`.
- Verify the base is syncing, **from the Pi** (needs a TTY — use `ssh -t` if
  running from the laptop):
  ```
  sudo chronyc clients
  ```
  Want `<CREATE3_USB_IP>` with a non-zero NTP count.
- ⚠️ **After any large time step on the Pi, the base rejects the new time until
  it reboots.** Its log shows
  `reply from <PI_USB_IP>: delay ### is too high, ignoring`.
  Reboot the base via the webserver, or hold the button until the light ring
  fully extinguishes. A `sudo reboot` on the Pi does **not** reset the base.

**After any clock fix:** fully restart AMCL, Nav2 and RViz. Stale timestamps
poison the tf buffers and do not self-clear.

### 2026-08-10 — Nav2 partial-bringup failure (and why bringup order matters)

`lifecycle_manager` configures and activates its managed nodes in strict
sequence, and **aborts the remainder if any one node fails or times out**:

```
Failed to bring up all requested nodes. Aborting bringup.
```

Clock/tf problems cause exactly this. The result is a stack that *looks* partly
alive — `controller_server` active, but `bt_navigator`, `planner_server`,
`velocity_smoother` and `collision_monitor` all `inactive` or `unconfigured` —
and goals silently do nothing.

- **Root cause:** several Nav2 nodes wait on tf during their configure/activate
  transition. With no valid `map → odom`, those waits time out and the manager
  aborts. Starting Nav2 *before* localization has a good pose is the trigger.
- **Lesson:** follow the startup sequence at the top of this file. Verify each
  layer before starting the next.
- **Do not hand-activate** with `ros2 lifecycle set` except for debugging — it
  skips the manager's bond monitoring, so dead nodes will not be restarted.
- Added two verification functions to `~/.bashrc`:
  - `navcheck` — prints the lifecycle state of every node the navigation
    lifecycle manager owns.
  - `loccheck` — checks `map → odom` exists and compares the `/odom` stamp
    against wall clock.

### 2026-08-10 — Nav2 `cmd_vel` chain (trace this when the robot won't move)

```
controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed
  → collision_monitor → /cmd_vel → create3_repub
```

Diagnose by running `ros2 topic hz` on each link with a goal active. **The first
silent one is the break.** Two real bugs found this way:

1. **`collision_monitor` transform tolerance was far too tight.** Lab Wi-Fi
   jitter of only ~200 **microseconds** between the scan and odom stamps was
   enough to trigger
   `Robot to stop due to invalid source`, which surfaces downstream as
   `Failed to make progress`. Fixed with `transform_tolerance: 0.5` and
   `source_timeout: 2.0`.
2. **`controller_frequency` was 20 Hz** but the lab link only delivers 6–20 Hz,
   so MPPI output stuttered. Set to `10.0`.
   ⚠️ **MPPI requires `model_dt` to match the controller period.** Changing
   frequency to 10 Hz without also setting `model_dt: 0.1` produces
   `Controller period more then model dt` and aborts bringup.

### 2026-08-10 — Local Nav2 params file created

All of the above were initially applied with `ros2 param set`, which **does not
survive a restart**. Copied the stock config and baked the fixes in:

```
cp /opt/ros/jazzy/share/turtlebot4_navigation/config/nav2.yaml \
   ~/turtlebot4_ws/config/nav2_lab.yaml
```

Changes from stock:

| Block | Key | Stock | Now |
|---|---|---|---|
| `controller_server` | `controller_frequency` | 20.0 | **10.0** |
| `controller_server` | `model_dt` (MPPI) | 0.05 | **0.1** |
| `controller_server` | `progress_checker.required_movement_radius` | 0.5 | **0.25** |
| `controller_server` | `progress_checker.movement_time_allowance` | 10.0 | **20.0** |
| `collision_monitor` | `transform_tolerance` | 0.2 | **0.5** |
| `collision_monitor` | `source_timeout` | 1.0 | **2.0** |

Also appended a `lifecycle_manager_navigation` block (`bond_timeout: 20.0`,
`attempt_respawn_reconnection: true`) because bringup was timing out under
laptop load.

Launch with (full path — `~` does not expand inside launch files):
```
ros2 launch turtlebot4_navigation nav2.launch.py \
  params_file:=$HOME/turtlebot4_ws/config/nav2_lab.yaml
```

Known-good copy preserved at `nav2_lab.yaml.working`.

⚠️ Leave `docking_server`'s own `controller_frequency: 50.0` alone — it is
unrelated to navigation.

### 2026-08-10 — Costmap and RViz gotchas

- **All-zero `/cmd_vel` with a goal active** almost always means the robot is
  sitting **inside a lethal-cost inflation blob** and has no viable trajectory.
  Confirmed today: physically moving the robot clear and re-setting the 2D Pose
  Estimate fixed it immediately. Diagnose fastest by adding
  `/local_costmap/costmap` as a Map display in RViz and checking whether the
  robot is in the red. Clear with:
  ```
  ros2 service call /local_costmap/clear_entirely_local_costmap \
    nav2_msgs/srv/ClearEntireCostmap "{}"
  ```
  (and the `/global_costmap/clear_entirely_global_costmap` equivalent).
- **RViz Map display shows Width/Height/Resolution all 0.** `map_server`
  publishes `TRANSIENT_LOCAL` but the RViz Map display defaults to
  `Reliable`/`Volatile`. If RViz starts before the map is latched it sees
  nothing. Fix: expand the Topic row, set **Durability = Transient Local**.
- **`map:=~/path/...` does not tilde-expand** inside launch files. Always pass
  the full `$HOME/...` path.
- **The 2D Pose Estimate tool stamps clicks with the current Fixed Frame**, so
  Fixed Frame must be set to `map` or AMCL will never see a valid pose.
- **Amcl Particle Swarm display never appears:** AMCL publishes `BEST_EFFORT`
  while the display asks for `RELIABLE`. Set the display to Best Effort.
- **The RPLiDAR A1 sits at z = 0.193 m and cannot see tabletops.** The robot hit
  a table today. Inflation is the main protection against this. Prefer keeping
  `inflation_radius` around 0.4 with `cost_scaling_factor` 8–10 over lowering
  the radius, or black out furniture footprints directly in `room_a.pgm` for the
  demo.

### 2026-08-10 — Status at end of session

Nav2 confirmed working end to end: full lifecycle stack active, robot
navigating to RViz goals, all three clocks in agreement.

### 2026-08-12 — OAK-D image transport: raw topics are unusable over Wi-Fi

Measured while building the depth-based object localization. This determines
which topics every downstream node must subscribe to.

**The raw image topics deliver 0 Hz to the workstation.**

| Topic | On the Pi | At the laptop | Size/frame |
|---|---|---|---|
| `/oakd/rgb/image_raw` | 4.1 Hz | **0.0 Hz** | 2.7 MB |
| `/oakd/stereo/image_raw` | 10.5 Hz | **0.0 Hz** | 1.8 MB |
| `/oakd/rgb/image_raw/compressed` | 14.6 Hz | works | ~177 KB |
| `/oakd/stereo/image_raw/compressedDepth` | — | works | ~355 KB |

The camera is healthy; the network is the limit. Frames that size fragment
across many UDP packets and effectively never arrive intact. Practical TCP
throughput to the Pi measured **3.3 MB/s**, against ~11 MB/s needed for raw RGB
alone.

- **Use the compressed topics.** `compressedDepth` is lossless PNG behind a
  short binary header (find the PNG magic rather than assuming 12 bytes). It
  decodes to exactly the uint16 1280×720 millimetre image the raw topic carries,
  so the RGB intrinsics apply unchanged and a bbox pixel indexes straight in.
  Verified: 84% valid pixels, range 632–9810 mm.
- ⚠️ **These topics are published `RELIABLE`, and subscribers must match.**
  Using `qos_profile_sensor_data` (BEST_EFFORT) yields **zero** frames —
  measured 0 synced pairs in 40 s. A multi-fragment sample that loses one
  fragment is dropped with no retransmit; RELIABLE asks for it again. This is
  the opposite of the usual "sensor topics want BEST_EFFORT" advice, and the
  opposite of what was assumed when this work started.
- RGB and depth from the RGBD pipeline share timestamps **exactly** (0 ms
  delta), but RGB arrives ~3× more often, so an
  `ApproximateTimeSynchronizer` is still needed to pick the frames that have a
  depth partner. Yields ~2 Hz of pairs — ample for a stop-and-look search.

### 2026-08-12 — The OAK-D does not stream while docked

Detection appears silently broken on the dock: `camera_info` still publishes, so
the node looks alive, but no image frames arrive. Undock first.

```
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

`mission` now reads `/dock_status` and offers to undock; `detect_3d`'s heartbeat
names the dock as the likely cause when it has never received a frame, and
distinguishes that from a stream that was working and then stalled.

### 2026-08-12 — Object localization pipeline implemented

Depth-based, as anticipated in the previous outstanding list. Detection →
median depth patch at the bbox centre (5×5, zeros rejected, widening once if the
patch is mostly holes) → back-project through the intrinsics → tf2 into `map`.

- Intrinsics read from `/oakd/rgb/camera_info`:
  `fx 1027.002, fy 1026.493, cx 635.739, cy 373.907`, distortion all zero.
- Validated against ground truth: a chair 2.61 m down the optical axis resolves
  to x = +2.55 m in `base_link`, matching the camera's 0.060 m forward offset
  exactly.
- Depth accepted only in the 0.3–6.0 m band; OAK-D stereo is not trustworthy
  beyond that and produces plausible-looking wrong numbers rather than failing.
- ⚠️ **A single pixel is not a depth reading.** Dark, shiny and textureless
  surfaces return 0. The centre of a black office chair is full of holes, hence
  the median-of-valid-pixels patch.

New nodes in `object_detector`: `detect_3d`, `search`, `mission`. See
[`README.md`](README.md).

---

## Outstanding / not yet done

- [ ] **Move Nav2 onto the Pi.** Currently the whole stack runs on the laptop,
  so every costmap update, control cycle and collision check crosses jittery lab
  Wi-Fi. This is the single largest remaining source of instability — the
  control loop was observed swinging between 6.6 and 20 Hz. Running Nav2 on the
  Pi with RViz only on the laptop puts sensors, costmaps and controller on
  localhost. **Do this before layering the detection pipeline on top.**
- [ ] **Fix the `navcheck` grep** — change `grep -oP "'\K[^']+"` to
  `grep -oP "'\K[^',]+"` so it stops parsing commas as node names.
- [ ] **Repeatability testing.** Run 4+ goals to different parts of the room and
  record the success rate. Needed before the handover demo.
- [x] **Confirm OAK-D depth stream and camera intrinsics are publishing.**
  Done 2026-08-12 — depth-based localization implemented. See the entries above.
- [x] **Waypoint coverage strategy.** Done — `kmeans_waypoint_tour.py` visits the
  poses in `config/search_waypoints.yaml` (five, derived from the map's free
  space) and sweeps a full circle in four steps at each, since the OAK-D FOV is
  narrow. The strategy is now pluggable: shared machinery sits in
  `search_common.py`, strategies are registered in `search_algorithms.py`, and
  the algorithm is chosen at the `mission` prompt or via `algorithm:=`.
- [x] **Detection node with pose capture.** Done — `detect_3d` transforms each
  detection in the same callback that received the image, using the image's own
  timestamp, so the camera pose that produced the pixel is the one used.
- [ ] **Run the full search on hardware.** The Nav2 drive itself
  (`goToPose`, per-waypoint sweep, found/not-found report) has not yet been
  exercised on the robot. Expect to tune `DWELL_SECONDS`, `MIN_SIGHTINGS` and
  `CLUSTER_RADIUS` in `search_common.py` on the first real run.
- [ ] **Investigate the `/oakd` node disappearing.** Observed 2026-08-12: the
  camera driver vanished from `ros2 node list` mid-session (publisher count
  dropped to 0, not merely slow), producing bursts of frames separated by long
  silences. Suspect a brownout on a low battery, since an OAK-D Pro draws real
  current. Check `journalctl -u turtlebot4.service` on the Pi for USB/XLink
  errors or an OOM kill next time it happens.
- [ ] **Furniture the lidar cannot see** — either black out footprints in
  `room_a.pgm` or add the OAK-D depth cloud as a second costmap observation
  source.