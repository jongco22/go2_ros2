#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener, TransformException


class StraightMoveNode(Node):
    def __init__(self) -> None:
        super().__init__('straight_move')

        self.declare_parameter('target_distance_m', 2.0)
        self.declare_parameter('linear_speed_mps', 0.5)
        self.declare_parameter('distance_tolerance_m', 0.02)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.target_distance_m = float(self.get_parameter('target_distance_m').value)
        self.linear_speed_mps = float(self.get_parameter('linear_speed_mps').value)
        self.distance_tolerance_m = float(self.get_parameter('distance_tolerance_m').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_publisher = self.create_publisher(Bool, '/mission_straight_driving_feedback', 10)
        self.trigger_subscriber = self.create_subscription(
            Bool,
            '/mission_straight_driving',
            self._trigger_callback,
            10,
        )

        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.mission_active = False
        self.mission_completed = False
        self.start_position_set = False
        self.start_x = 0.0
        self.start_y = 0.0

        self.get_logger().info('StraightMoveNode initialized')

    def _trigger_callback(self, msg: Bool) -> None:
        if msg.data:
            if not self.mission_active:
                self.mission_active = True
                self.mission_completed = False
                self.start_position_set = False
                self.get_logger().info('Straight driving mission started')
            else:
                self.get_logger().debug('Mission already active; ignoring start signal')
        else:
            if self.mission_active:
                self.get_logger().info('Straight driving mission cancelled')
            self._stop_robot()
            self.mission_active = False
            self.mission_completed = False
            self.start_position_set = False

    def _control_loop(self) -> None:
        if not self.mission_active:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return

        current_x = float(transform.transform.translation.x)
        current_y = float(transform.transform.translation.y)

        if not self.start_position_set:
            self.start_x = current_x
            self.start_y = current_y
            self.start_position_set = True
            self.get_logger().info(
                f'Straight mission reference set at x={self.start_x:.3f}, y={self.start_y:.3f}'
            )

        distance_travelled = math.hypot(current_x - self.start_x, current_y - self.start_y)

        if distance_travelled + self.distance_tolerance_m < self.target_distance_m:
            twist = Twist()
            twist.linear.x = float(self.linear_speed_mps)
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
        else:
            self._stop_robot()
            if not self.mission_completed:
                self.feedback_publisher.publish(Bool(data=True))
                self.get_logger().info(
                    f'Straight driving mission completed: {distance_travelled:.3f} m'
                )
                self.mission_completed = True
            self.mission_active = False
            self.start_position_set = False

    def _stop_robot(self) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down StraightMoveNode')
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


