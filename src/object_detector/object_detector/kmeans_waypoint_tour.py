#!/usr/bin/env python3
"""
kmeans_waypoint_tour.py — visit k-means room-coverage waypoints in file order.

This is the original search, and the baseline the others are measured against.

The waypoints are not guessed. config/search_waypoints.yaml was built from the
occupancy grid itself: the free space of room_a.pgm was distance-transformed to
keep at least 0.45 m of clearance from any wall or unknown cell, then k-means
split the reachable area (44.6 m^2) into five clusters. Each waypoint is a
cluster centroid, so the set covers the room roughly evenly and no stop is
wedged against a wall.

The tour then walks that list top to bottom, sweeping a full turn at each stop,
and returns as soon as one cluster of sightings is convincing.

What this strategy assumes: nothing. It has no belief about where the object is
likely to be, and no memory between runs. Every run drives the same path in the
same order, which makes it a clean control condition — and an obvious thing to
beat.
"""

from object_detector.search_common import drive_to, sweep_in_place

NAME = "kmeans-tour"
TITLE = "K-means waypoint tour (baseline)"
DESCRIPTION = (
    "Visits the five k-means room-coverage waypoints in file order, sweeping a "
    "full turn at each. Deterministic, covers the room evenly, ignores where "
    "the object is likely to be."
)


def run(nav, collector, waypoints, target):
    for i, wp in enumerate(waypoints, 1):
        if not drive_to(nav, collector, wp, label=f"waypoint {i}/{len(waypoints)}: "):
            continue
        hit = sweep_in_place(nav, collector)
        if hit:
            return hit
    return None
