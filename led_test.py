#!/usr/bin/env python3
"""
led_test.py — minimal ROS 2 node that commands the TurtleBot 4 light ring.

Why this script exists:
    To confirm that commands issued from a *script* (rather than typed by hand
    into the terminal) actually reach the robot. The light ring is the ideal
    safe target — no motion, it works while docked, and the result is instantly
    visible. If the ring turns green, the whole path works:
        this script -> rclpy -> DDS -> the Create 3's /cmd_lightring subscriber.

Run it (with ROS 2 sourced), from the laptop or the Pi:
    python3 led_test.py
"""

import time
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import LightringLeds, LedColor


class LightringTest(Node):
    def __init__(self):
        # Every ROS 2 node has a name; this is how it shows up in `ros2 node list`.
        super().__init__("lightring_test")
        # A publisher: what topic, what message type, and a queue depth of 10.
        self.pub = self.create_publisher(LightringLeds, "/cmd_lightring", 10)

    def set_ring(self, r, g, b, override):
        """Build one light-ring message and publish it."""
        msg = LightringLeds()
        msg.header.stamp = self.get_clock().now().to_msg()
        # override_system = True  -> our script takes control of the ring
        # override_system = False -> hand control back to the robot's own display
        msg.override_system = override
        # The ring has 6 LEDs; set all of them to the same colour (0-255 each).
        msg.leds = [LedColor(red=r, green=g, blue=b) for _ in range(6)]
        self.pub.publish(msg)


def main():
    rclpy.init()                      # start up ROS 2 for this process
    node = LightringTest()

    node.get_logger().info("Turning the ring GREEN for 5 seconds...")
    # We publish repeatedly rather than once: the very first message after
    # startup is usually dropped because DDS "discovery" (the robot finding our
    # publisher) hasn't finished yet. Sending for a few seconds guarantees it lands.
    t_end = time.time() + 5.0
    while time.time() < t_end:
        node.set_ring(0, 255, 0, override=True)   # green, we control the ring
        time.sleep(0.1)                           # 10 times per second

    node.get_logger().info("Releasing the ring back to the robot.")
    node.set_ring(0, 0, 0, override=False)        # give control back
    time.sleep(0.2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()