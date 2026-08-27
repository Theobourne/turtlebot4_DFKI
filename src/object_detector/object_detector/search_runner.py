#!/usr/bin/env python3
"""
search_runner.py — run a chosen search algorithm and report what it found.

This is the `search` executable. It owns everything that is the same whichever
strategy runs: bring-up, handing the strategy the robot, then recording the
find, saving proof, docking and printing one summary.

    ros2 launch object_detector search.launch.py target_class:=chair \\
        algorithm:=kmeans-tour

or, with detect_3d already running:

    ros2 run object_detector search --ros-args \\
        -p target_class:=chair -p algorithm:=nearest-first

Algorithms live in their own modules and are listed in search_algorithms.py.
Run with an unknown name to be told which ones exist.
"""

import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

from object_detector import search_algorithms
from object_detector import object_map
from object_detector.object_map import record_object
from object_detector.search_common import (
    MIN_SIGHTINGS, RETURN_TO_DOCK, SightingCollector, TargetFound, go_home,
    load_waypoints,
)


def main(args=None):
    rclpy.init(args=args)
    collector = SightingCollector()
    collector.declare_parameter("target_class", "object")
    collector.declare_parameter("algorithm", search_algorithms.DEFAULT)
    collector.declare_parameter("continuous_sensing", True)
    # 'replace' answers "where is it now" — right while carrying one object
    # around to test. 'merge' accumulates, for rooms with several.
    collector.declare_parameter("object_map_mode", object_map.DEFAULT_MODE)
    target = collector.get_parameter("target_class").value
    algo_name = collector.get_parameter("algorithm").value
    collector.continuous = collector.get_parameter("continuous_sensing").value

    algorithm = search_algorithms.get(algo_name)
    if algorithm is None:
        # Fail loudly and immediately. A typo that silently fell back to the
        # default would quietly invalidate a comparison between strategies.
        collector.get_logger().error(
            f"unknown algorithm '{algo_name}'. Available: "
            f"{', '.join(search_algorithms.names())}"
        )
        collector.destroy_node()
        rclpy.shutdown()
        return 2

    nav = BasicNavigator()
    waypoints = load_waypoints(collector)

    collector.get_logger().info("waiting for Nav2 to come up...")
    nav.waitUntilNav2Active()
    collector.get_logger().info(
        f"Nav2 active — searching for '{target}' using {algorithm.TITLE}"
    )
    # Said out loud because it changes what the run means. Continuous sensing
    # can find the target between waypoints, which makes every strategy look
    # more alike — turn it off when comparing them.
    collector.get_logger().info(
        "continuous sensing ON — will interrupt driving to check anything promising"
        if collector.continuous else
        "continuous sensing OFF — looking only at waypoints"
    )

    found = None
    image_path = None
    near_miss = None
    docked = False
    went_home = False
    started = time.time()
    try:
        try:
            found = algorithm.run(nav, collector, waypoints, target)
        except TargetFound as interrupt:
            # Spotted and verified between waypoints rather than at one.
            found = interrupt.cluster

        if found:
            x, y, z = found["centre"]
            n = len(found["pts"])
            # Save proof of the find before moving: the robot is about to drive
            # to the dock and the object will leave the frame.
            image_path = collector.save_find(target, found, n)
            # Put it on the map permanently. Merges with a previous find of the
            # same class nearby rather than stacking duplicates.
            recorded, action = record_object(
                target, x, y, z, sightings=n,
                mode=collector.get_parameter("object_map_mode").value)
            collector.get_logger().info(
                f"{action} '{target}' on the object map at "
                f"({recorded['x']:+.2f}, {recorded['y']:+.2f}) "
                f"from {recorded['sightings']} sightings"
            )
            for _ in range(10):
                collector.publish_result(found["centre"], f"{target} ({n} sightings)")
                time.sleep(0.05)
        elif collector.clusters:
            best = max(collector.clusters, key=lambda c: len(c["pts"]))
            near_miss = (f"best guess had only {len(best['pts'])} sightings, "
                         f"needed {MIN_SIGHTINGS}")

        # --- go home ---------------------------------------------------------
        # Do this before the final report so the last thing printed is the
        # outcome, not a trail of docking chatter.
        if RETURN_TO_DOCK:
            # Nav2 covers the distance, docking covers the last metre. Calling
            # dock() from wherever the object happened to be fails: the Dock
            # action only searches its immediate surroundings for the beacon.
            went_home = go_home(nav, collector)
            docked = collector.dock()

    except KeyboardInterrupt:
        print("\nSearch interrupted.")
    finally:
        # One summary, printed last, on plain stdout rather than the logger —
        # otherwise it competes with detect_3d's detection stream for the final
        # word and gets scrolled away.
        elapsed = time.time() - started
        line = "=" * 62
        print(f"\n{line}")
        if found:
            x, y, z = found["centre"]
            print(f"  SUCCESS — found '{target}'")
            print(f"  map coordinates : x={x:+.2f}  y={y:+.2f}  z={z:+.2f}")
            print(f"  confidence      : {len(found['pts'])} agreeing sightings")
            v = found.get("verified")
            if v:
                how = "after sidestepping" if v["sidestepped"] else "from the approach"
                print(f"  verified        : {how}, "
                      f"{v['baseline']:.2f} m between viewpoints")
            if image_path:
                print(f"  image           : {image_path}")
        else:
            print(f"  FAILURE — '{target}' not found")
            if near_miss:
                print(f"  {near_miss}")
        # Printed on every run, found or not: comparing strategies is the point
        # of having more than one, and that comparison needs a time.
        print(f"  algorithm       : {algorithm.NAME}"
              f"{'  + continuous sensing' if collector.continuous else ''}")
        print(f"  search time     : {elapsed:.0f}s")
        if collector.verifications:
            print(f"  cues checked    : {collector.verifications} "
                  f"({len(collector.blacklist)} rejected)")
        if RETURN_TO_DOCK:
            # Separate the two halves: driving home and docking fail for
            # different reasons, and "NOT docked" alone never said which.
            print(f"  return home     : {'reached start pose' if went_home else 'DID NOT reach start pose'}")
            print(f"  dock            : {'docked' if docked else 'NOT docked'}")
        print(f"{line}\n")

        collector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
