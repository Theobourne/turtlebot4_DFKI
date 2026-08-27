#!/usr/bin/env python3
"""
nearest_waypoint_tour.py — same waypoints, greedy nearest-first order.

Identical coverage to the k-means baseline, identical sweep at each stop; the
only difference is which stop comes next. Instead of following the file order,
it repeatedly drives to whichever unvisited waypoint is closest to where the
robot actually is.

Why bother: the baseline's order is an artefact of how k-means happened to
number its clusters, so it can zig-zag across the room. Driving is by far the
slowest part of a search, and time-to-find is what matters when the object is
found early — which it usually is. Cutting travel between stops cuts the cost of
every waypoint the robot visits before it gets lucky.

What it does not do: change *where* it looks, only the order. If the object sits
at the last waypoint either way, this saves nothing. It is a travel-cost
optimisation, not a smarter search — which makes it a fair A/B against the
baseline, since coverage is held constant.
"""

import math

from object_detector.search_common import drive_to, sweep_in_place

NAME = "nearest-first"
TITLE = "Nearest-first waypoint tour"
DESCRIPTION = (
    "Same waypoints and sweep as the baseline, but always drives to the "
    "closest unvisited one. Same coverage, less travel."
)


def run(nav, collector, waypoints, target):
    remaining = list(waypoints)
    total = len(remaining)
    here = collector.current_xy()
    visited = 0

    while remaining:
        wp = min(remaining,
                 key=lambda w: math.hypot(w["x"] - here[0], w["y"] - here[1]))
        remaining.remove(wp)
        visited += 1

        reached = drive_to(
            nav, collector, wp,
            label=f"waypoint {visited}/{total} (nearest, "
                  f"{math.hypot(wp['x'] - here[0], wp['y'] - here[1]):.1f} m): ")

        # Plan the next leg from where the robot really ended up, not from where
        # it was aimed — those differ after a Nav2 recovery, and they differ a
        # lot when the goal was abandoned.
        here = collector.current_xy(fallback=(wp["x"], wp["y"]))

        if not reached:
            continue
        hit = sweep_in_place(nav, collector)
        if hit:
            return hit
    return None
