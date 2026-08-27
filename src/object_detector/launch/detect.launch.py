# detect.launch.py — undock the robot, then run the detector, in one command.
#
# The undock runs as its own process and the detector as its own node, so the
# detector starts regardless of whether the undock cleanly reports "done"
# (which is what made the all-in-one-node version hang).
#
# *** Undocking drives the robot BACKWARD off the dock — clear space behind it. ***
#
#   ros2 launch object_detector detect.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Step 1: send the undock command (runs as a normal process, then exits).
        ExecuteProcess(
            cmd=[
                "ros2", "action", "send_goal", "/undock",
                "irobot_create_msgs/action/Undock", "{}",
            ],
            output="screen",
        ),

        # Step 2: the detector node. Starts right away; it will simply wait for
        # the first camera frame (which arrives once the robot is off the dock).
        Node(
            package="object_detector",
            executable="detect",
            output="screen",
        ),
    ])