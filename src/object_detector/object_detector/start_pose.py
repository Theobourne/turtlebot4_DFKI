#!/usr/bin/env python3
"""
start_pose.py — where the robot starts, remembered between runs.

Shared by mission (which records it and seeds AMCL from it) and the search
(which drives back to it before docking). It lives in its own module so the
search does not have to import mission, which pulls in ultralytics and the
whole interactive front door for the sake of two functions.
"""

import os
import time

import yaml

START_POSE_FILE = os.path.join(
    os.path.expanduser("~"), "turtlebot4_ws", "config", "start_pose.yaml"
)


def load_start_pose(path=None):
    """The recorded starting pose, as a dict, or None.

    Returns x/y/yaw plus the context it was recorded in, because a pose is only
    meaningful if the robot is actually back in that spot.
    """
    try:
        with open(path or START_POSE_FILE) as f:
            p = yaml.safe_load(f)["start_pose"]
        return {
            "x": float(p["x"]), "y": float(p["y"]), "yaw": float(p["yaw"]),
            # Older files predate these fields; unknown is honest.
            "docked": p.get("docked"),
            "recorded": p.get("recorded", "unknown"),
        }
    except Exception:
        return None


def save_start_pose(x, y, yaw, docked=None, path=None):
    """Record the pose along with the dock state it was taken in.

    Undocking reverses the robot roughly 0.3 m, so a pose recorded docked is
    simply wrong for an undocked robot and vice versa. Storing the dock state
    lets the next run notice the mismatch instead of quietly seeding AMCL with
    a pose that is a third of a metre out.
    """
    with open(path or START_POSE_FILE, "w") as f:
        f.write(
            "# Where the robot sits when a mission begins — normally the dock.\n"
            "# Recorded by 'ros2 run object_detector mission' after you localized\n"
            "# by hand in RViz. Having this lets the next run set AMCL's initial\n"
            "# pose itself, which is the one genuinely interactive step in\n"
            "# bringing up localization.\n"
            "#\n"
            "# It is also where the search drives back to before docking: the Dock\n"
            "# action only looks for the dock nearby, so it fails from across the\n"
            "# room. Nav2 knows the way; docking only has to cover the last metre.\n"
            "#\n"
            "# 'docked' records whether the robot was on the dock at the time.\n"
            "# Undocking moves it ~0.3 m, so a pose taken in the other state is\n"
            "# wrong by about that much.\n"
            "#\n"
            "# Re-record it if the dock moves, or if the map is rebuilt.\n\n"
        )
        yaml.safe_dump({"start_pose": {
            "x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4),
            "docked": docked,
            "recorded": time.strftime("%Y-%m-%d %H:%M"),
        }}, f, sort_keys=False)
