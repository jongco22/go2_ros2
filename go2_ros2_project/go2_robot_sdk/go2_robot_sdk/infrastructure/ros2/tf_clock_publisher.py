#!/usr/bin/env python3
# Node used to provide /clock when running on a real robot with use_sim_time=true.
# /clock is required to use the global_costmap topic.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


class TfClockPublisher(Node):
    def __init__(self):
        super().__init__("tf_clock_publisher")

        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("clock_topic", "/clock")

        tf_topic = self.get_parameter("tf_topic").value
        clock_topic = self.get_parameter("clock_topic").value

        tf_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(Clock, clock_topic, clock_qos)
        self._sub = self.create_subscription(
            TFMessage, tf_topic, self._tf_cb, tf_qos
        )

        self._last_clock = None
        self.get_logger().info(f"Publishing /clock from {tf_topic}")

    def _tf_cb(self, msg: TFMessage):
        if not msg.transforms:
            return
        # Use the newest stamp in the TF message.
        latest = max(
            msg.transforms, key=lambda t: (t.header.stamp.sec, t.header.stamp.nanosec)
        )
        latest_stamp = latest.header.stamp
        if self._last_clock is not None:
            last = self._last_clock
            if (latest_stamp.sec, latest_stamp.nanosec) < (last.sec, last.nanosec):
                # Ignore backward jumps to keep /clock monotonic.
                return
        clock_msg = Clock()
        clock_msg.clock = latest_stamp
        self._pub.publish(clock_msg)
        self._last_clock = latest_stamp


def main(args=None):
    rclpy.init(args=args)
    node = TfClockPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()