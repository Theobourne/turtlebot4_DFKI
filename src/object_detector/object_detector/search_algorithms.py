#!/usr/bin/env python3
"""
search_algorithms.py — the list of search strategies you can choose between.

To add one: write a module next to this file exposing NAME, TITLE, DESCRIPTION
and run(nav, collector, waypoints, target), then add it to ALGORITHMS below.
Nothing else needs to change — the menu in mission.py, the --algorithm
parameter and the launch file all read this registry.
"""

from object_detector import kmeans_waypoint_tour, nearest_waypoint_tour

# Order matters: this is the order the menu offers them in, and the first entry
# is the default.
_MODULES = [
    kmeans_waypoint_tour,
    nearest_waypoint_tour,
]

ALGORITHMS = {m.NAME: m for m in _MODULES}
DEFAULT = _MODULES[0].NAME


def get(name):
    """Look up an algorithm by name, or None if there is no such thing."""
    return ALGORITHMS.get(name)


def names():
    return list(ALGORITHMS)
