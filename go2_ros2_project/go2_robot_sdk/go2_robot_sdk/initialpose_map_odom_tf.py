#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros


def quat_to_matrix(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(4)
    s = 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    m = np.eye(4)
    m[0, 0] = 1.0 - (yy + zz)
    m[0, 1] = xy - wz
    m[0, 2] = xz + wy
    m[1, 0] = xy + wz
    m[1, 1] = 1.0 - (xx + zz)
    m[1, 2] = yz - wx
    m[2, 0] = xz - wy
    m[2, 1] = yz + wx
    m[2, 2] = 1.0 - (xx + yy)
    return m


def matrix_to_quat(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    return x/norm, y/norm, z/norm, w/norm


def pose_to_matrix(pose):
    q = pose.orientation
    m = quat_to_matrix(q.x, q.y, q.z, q.w)
    m[0, 3] = pose.position.x
    m[1, 3] = pose.position.y
    m[2, 3] = pose.position.z
    return m


class InitialPoseMapOdom(Node):
    def __init__(self):
        super().__init__("initialpose_map_odom_tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.map_to_odom = np.eye(4)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10)
        self.create_timer(0.05, self.publish_tf)
        self.get_logger().info("Listening to /initialpose and publishing map -> odom. Use RViz 2D Pose Estimate.")

    def on_initialpose(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except Exception as exc:
            self.get_logger().error(f"Cannot lookup {self.odom_frame} -> {self.base_frame}: {exc}")
            return
        desired_map_base = pose_to_matrix(msg.pose.pose)
        q = tf.transform.rotation
        odom_base = quat_to_matrix(q.x, q.y, q.z, q.w)
        odom_base[0, 3] = tf.transform.translation.x
        odom_base[1, 3] = tf.transform.translation.y
        odom_base[2, 3] = tf.transform.translation.z
        self.map_to_odom = desired_map_base @ np.linalg.inv(odom_base)
        self.get_logger().info(f"Updated map->odom: base_link set to map x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}")
        self.publish_tf()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(self.map_to_odom[0, 3])
        t.transform.translation.y = float(self.map_to_odom[1, 3])
        t.transform.translation.z = float(self.map_to_odom[2, 3])
        q = matrix_to_quat(self.map_to_odom)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = InitialPoseMapOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
