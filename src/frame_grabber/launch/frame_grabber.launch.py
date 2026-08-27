# frame_grabber.launch.py — start the frame grabber with `ros2 launch`.
# Later you can add more nodes here so ONE command brings up a whole trial.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="frame_grabber",
            executable="grab_frames",
            name="frame_grabber",
            output="screen",
        ),
        # Add more nodes here as the project grows, e.g. the policy node,
        # the travel-cost tracker, etc. They'll all start together.
    ])