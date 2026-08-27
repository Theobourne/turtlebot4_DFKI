#!/usr/bin/env bash
# record_trial.sh — capture a TurtleBot 4 search trial to an MCAP bag.
#
# Run this ON THE ROBOT (ssh ubuntu@<ROBOT_IP>), NOT the laptop, so the
# camera stream never crosses the jittery Wi-Fi link and drops frames.
#
# Usage: ./record_trial.sh <room> <method> [trial_no]
#   e.g. ./record_trial.sh officeA oppcost 03
#
# Bags land in ~/trials/<stamp>_<room>_<method>_t<trial>/ (or /media/usb/... if a
# stick is mounted). Copy them to the laptop afterward for analysis / handover.

set -euo pipefail

ROOM="${1:?need a room name}"
METHOD="${2:?need a method name, e.g. oppcost|seek|threshold}"
TRIAL="${3:-01}"

STAMP="$(date +%Y%m%d_%H%M%S)"
NAME="${STAMP}_${ROOM}_${METHOD}_t${TRIAL}"

# Prefer a USB stick if one is mounted, to spare the boot SD card.
if mountpoint -q /media/usb 2>/dev/null; then
  OUTDIR="/media/usb/trials/${NAME}"
else
  OUTDIR="${HOME}/trials/${NAME}"
fi

# --- topics ---------------------------------------------------------------
# Lightweight state + odometry + lidar (cheap; always record)
STATE_TOPICS=(
  /tf /tf_static
  /odom /joint_states
  /scan
  /battery_state /diagnostics
  /cmd_vel
  /robot_description
)

# Camera. Compressed RGB is ~100-200 kB/frame vs ~2.7 MB raw at 720p.
# image_transport publishes /compressed lazily, so recording it triggers it.
# Add /oakd/stereo/image_raw or /oakd/points ONLY if you actually need depth —
# they are large.
CAM_TOPICS=(
  /oakd/rgb/image_raw/compressed
  /oakd/rgb/camera_info
)

# Nav2 (comment out the array contents if you are not running a full stack)
NAV_TOPICS=(
  /amcl_pose /plan /goal_pose /behavior_tree_log /map
)

# Your stopping / reallocation pipeline — uncomment once these topics exist.
# This is what turns the bag from "where the robot went" into "why it left
# each room": the room-type prior, the belief, the per-room value-per-cost
# estimate, and the leave events.
DECISION_TOPICS=(
  # /search/room_type
  # /search/belief
  # /search/value_estimate
  # /search/leave_event
)

ALL_TOPICS=(
  "${STATE_TOPICS[@]}"
  "${CAM_TOPICS[@]}"
  "${NAV_TOPICS[@]}"
  "${DECISION_TOPICS[@]}"
)

echo "Recording -> ${OUTDIR}"
echo "Topics:"
printf '  %s\n' "${ALL_TOPICS[@]}"
echo "Ctrl-C to stop."

# No bag-level compression: the heavy payload (images) is already JPEG, so
# zstd-ing the whole bag just burns Pi CPU. Add
#   --compression-mode file --compression-format zstd
# if you drop the camera and only record the small state/nav topics.
ros2 bag record \
  --storage mcap \
  --output "${OUTDIR}" \
  "${ALL_TOPICS[@]}"
