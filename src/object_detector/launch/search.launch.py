# search.launch.py — run the detector and the waypoint search together.
#
# Nav2 must already be up and localized (the robot needs a valid map -> odom
# transform) before this is useful. Bring it up first, e.g.
#   ros2 launch turtlebot4_navigation localization.launch.py map:=<...>/maps/room_a.yaml
#   ros2 launch turtlebot4_navigation nav2.launch.py params_file:=<...>/config/nav2_lab.yaml
#
#   ros2 launch object_detector search.launch.py target_class:=chair
#
# *** The robot drives itself around the room. Make sure the floor is clear. ***

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_class = LaunchConfiguration("target_class")
    waypoints_file = LaunchConfiguration("waypoints_file")
    algorithm = LaunchConfiguration("algorithm")
    model = LaunchConfiguration("model")
    continuous_sensing = LaunchConfiguration("continuous_sensing")
    object_map_mode = LaunchConfiguration("object_map_mode")

    default_waypoints = os.path.join(
        os.path.expanduser("~"), "turtlebot4_ws", "config", "search_waypoints.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "target_class",
            default_value="chair",
            description="COCO class name to search for, e.g. chair, bottle, person.",
        ),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=default_waypoints,
            description="YAML file of map-frame waypoints to visit.",
        ),
        DeclareLaunchArgument(
            "algorithm",
            default_value="kmeans-tour",
            description="Search strategy: see search_algorithms.py for the list.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="yolov8n.pt",
            description="YOLO weights. yolov8n is the only one fast enough "
                        "under real load; bigger models starve the search.",
        ),
        DeclareLaunchArgument(
            "object_map_mode",
            default_value="replace",
            description="'replace' drops earlier finds of the same class (right "
                        "when moving one object around); 'merge' accumulates.",
        ),
        DeclareLaunchArgument(
            "continuous_sensing",
            default_value="true",
            description="Look while driving and interrupt to check cues. "
                        "Set false when comparing search strategies.",
        ),

        # The eyes: YOLO + depth -> map-frame points.
        Node(
            package="object_detector",
            executable="detect_3d",
            output="screen",
            parameters=[{"target_class": target_class, "target_frame": "map",
                         "model": model}],
        ),

        # The legs: Nav2 waypoint tour that consumes those points.
        #
        # on_exit=Shutdown() is what makes the whole thing stop cleanly. Without
        # it the detector keeps running after the search finishes, and its
        # detection stream scrolls the result off the screen — the answer ends up
        # buried under a trail of "chair 0.7 @ ..." lines.
        Node(
            package="object_detector",
            executable="search",
            output="screen",
            parameters=[{
                "target_class": target_class,
                "waypoints_file": waypoints_file,
                "algorithm": algorithm,
                "continuous_sensing": continuous_sensing,
                "object_map_mode": object_map_mode,
            }],
            on_exit=Shutdown(reason="search finished"),
        ),
    ])
