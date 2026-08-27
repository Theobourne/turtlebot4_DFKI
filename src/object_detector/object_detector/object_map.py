#!/usr/bin/env python3
"""
object_map.py — a persistent semantic layer over the occupancy map.

The search already works out *where* an object is; this is what makes that
answer survive. Finds are written to config/object_map.yaml, and this node
publishes them as latched RViz markers, so the objects are on the map whenever
you look — including after the search has exited, and after RViz restarts.

    ros2 run object_detector object_map

Then in RViz: Add -> By topic -> /object_map/markers -> MarkerArray.

Two design choices worth knowing:

* The occupancy grid (room_a.pgm) is NOT modified. That file says where the
  walls are, and burning objects into it would corrupt the thing Nav2 plans
  against. The semantic layer belongs beside the map, not inside it.

* Markers are published TRANSIENT_LOCAL (latched), so a subscriber that
  connects later still receives them. A volatile publisher only reaches
  subscribers already listening, which is why the old result marker vanished
  whenever RViz was restarted.
"""

import os
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

# ---- settings you can change ---------------------------------------------
OBJECT_MAP_FILE = os.path.join(
    os.path.expanduser("~"), "turtlebot4_ws", "config", "object_map.yaml"
)
MARKER_TOPIC = "/object_map/markers"

# Two finds of the same class closer than this are treated as the same object
# and merged, rather than piling duplicates onto the map every run.
MERGE_RADIUS = 0.8

# What to do with an earlier find of the same class somewhere else.
#
#   "replace"  a new find of a class removes every previous entry of that class.
#              Right when you are carrying one chair around the room to test the
#              search: each run answers "where is it NOW", instead of leaving a
#              trail of everywhere it has ever been.
#
#   "merge"    nearby finds are averaged together (see MERGE_RADIUS) and distant
#              ones kept as separate objects. Right when the room genuinely
#              holds several of the thing.
DEFAULT_MODE = "replace"

RELOAD_PERIOD = 2.0        # how often to check the file for changes, seconds

# We republish on every tick, not only when the file changes. Latching alone is
# not enough: a subscriber that asks for VOLATILE durability still matches a
# TRANSIENT_LOCAL publisher, but is not given the already-published sample. RViz
# marker displays default to VOLATILE, so a display added after the node started
# would stay empty until the file next changed. Repeating a handful of markers
# every couple of seconds costs nothing and makes the display work regardless of
# what QoS the subscriber asked for, or when it connected.
# --------------------------------------------------------------------------


def load_objects(path=None):
    """Every object recorded so far, as a list of dicts."""
    path = path or OBJECT_MAP_FILE
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return data.get("objects", []) or []
    except FileNotFoundError:
        return []
    except Exception:
        return []


def save_objects(objects, path=None):
    path = path or OBJECT_MAP_FILE
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(
            "# Objects located by the search, in the map frame of maps/room_a.yaml.\n"
            "# Written by 'ros2 run object_detector search' and drawn by\n"
            "# 'ros2 run object_detector object_map'.\n"
            "#\n"
            "# This sits BESIDE the occupancy grid rather than inside it: room_a.pgm\n"
            "# describes walls for Nav2 to plan against, and must not be edited.\n"
            "#\n"
            "# Delete an entry to remove it from the map, or delete the file to\n"
            "# start over.\n\n"
        )
        yaml.safe_dump({"objects": objects}, f, sort_keys=False)


def record_object(name, x, y, z, sightings=1, confidence=None, path=None,
                  mode=None):
    """Record a find. Returns (object, action) where action says what happened:
    'added', 'updated' (merged with a nearby earlier find) or 'replaced'.

    Merging keeps a running average weighted by how many sightings each estimate
    rests on, so repeated searches sharpen the position rather than overwriting
    it with whatever the last run happened to see. See DEFAULT_MODE for when
    that is the wrong behaviour.
    """
    mode = mode or DEFAULT_MODE
    objects = load_objects(path)

    if mode == "replace":
        # Drop every earlier entry of this class, wherever it was. The object
        # has moved; its old positions are not additional objects, they are
        # stale.
        before = len(objects)
        objects = [o for o in objects if o["name"] != name]
        replaced = before - len(objects)
        obj = _new(name, x, y, z, sightings, confidence)
        objects.append(obj)
        save_objects(objects, path)
        return obj, ("replaced" if replaced else "added")

    for obj in objects:
        if obj["name"] != name:
            continue
        if (obj["x"] - x) ** 2 + (obj["y"] - y) ** 2 > MERGE_RADIUS ** 2:
            continue
        total = obj["sightings"] + sightings
        obj["x"] = (obj["x"] * obj["sightings"] + x * sightings) / total
        obj["y"] = (obj["y"] * obj["sightings"] + y * sightings) / total
        obj["z"] = (obj["z"] * obj["sightings"] + z * sightings) / total
        obj["sightings"] = total
        obj["updated"] = time.strftime("%Y-%m-%d %H:%M")
        if confidence is not None:
            obj["confidence"] = round(max(obj.get("confidence", 0.0), confidence), 2)
        for k in ("x", "y", "z"):
            obj[k] = round(float(obj[k]), 3)
        save_objects(objects, path)
        return obj, "updated"

    obj = _new(name, x, y, z, sightings, confidence)
    objects.append(obj)
    save_objects(objects, path)
    return obj, "added"


def _new(name, x, y, z, sightings, confidence):
    now = time.strftime("%Y-%m-%d %H:%M")
    return {
        "name": name,
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "z": round(float(z), 3),
        "sightings": int(sightings),
        "confidence": round(float(confidence), 2) if confidence is not None else None,
        "first_seen": now,
        "updated": now,
    }


class ObjectMap(Node):
    """Publishes the stored objects as latched markers, reloading on change."""

    def __init__(self):
        super().__init__("object_map")
        self.declare_parameter("object_map_file", OBJECT_MAP_FILE)
        self.path = self.get_parameter("object_map_file").value

        # Latched: RViz gets the markers whenever it connects, not only if it
        # happened to be listening at the moment we published.
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(MarkerArray, MARKER_TOPIC, qos)

        self.last_mtime = None
        self.objects = []
        self.create_timer(RELOAD_PERIOD, self.tick)
        self.reload_if_changed(first=True)
        self.publish(self.objects)

        self.get_logger().info(
            f"publishing {MARKER_TOPIC} from {self.path}\n"
            f"  In RViz: Add -> By topic -> {MARKER_TOPIC} -> MarkerArray"
        )

    def tick(self):
        self.reload_if_changed()
        self.publish(self.objects)

    def reload_if_changed(self, first=False):
        try:
            mtime = os.path.getmtime(self.path)
        except OSError:
            if first:
                self.get_logger().info("no objects recorded yet — nothing to draw.")
            return
        if mtime == self.last_mtime:
            return
        self.last_mtime = mtime
        self.objects = load_objects(self.path)
        self.get_logger().info(f"drawing {len(self.objects)} object(s) on the map.")

    def publish(self, objects):
        array = MarkerArray()

        # Clear first, so objects deleted from the file disappear from RViz
        # instead of lingering as ghosts.
        clear = Marker()
        clear.header.frame_id = "map"
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for i, obj in enumerate(objects):
            array.markers.append(self._sphere(obj, i))
            array.markers.append(self._label(obj, i))
        self.pub.publish(array)

    def _stamp(self, marker):
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_map"
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _sphere(self, obj, i):
        m = self._stamp(Marker())
        m.id = i * 2
        m.type = Marker.SPHERE
        m.pose.position.x = float(obj["x"])
        m.pose.position.y = float(obj["y"])
        m.pose.position.z = float(obj["z"])
        m.scale.x = m.scale.y = m.scale.z = 0.35
        m.color.g, m.color.a = 1.0, 0.85
        return m

    def _label(self, obj, i):
        m = self._stamp(Marker())
        m.id = i * 2 + 1
        m.type = Marker.TEXT_VIEW_FACING
        m.pose.position.x = float(obj["x"])
        m.pose.position.y = float(obj["y"])
        m.pose.position.z = float(obj["z"]) + 0.35
        m.scale.z = 0.22
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f"{obj['name']} ({obj['sightings']})"
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMap()
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
