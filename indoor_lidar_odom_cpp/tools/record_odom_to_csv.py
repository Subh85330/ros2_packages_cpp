#!/usr/bin/env python3
import math
import csv
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    # standard yaw from quaternion (assuming ENU / REP-103)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')

        # Create CSV files
        self.est_file = open('odom_est.csv', 'w', newline='')
        self.gt_file = open('odom_sim.csv', 'w', newline='')

        self.est_writer = csv.writer(self.est_file)
        self.gt_writer = csv.writer(self.gt_file)

        # Headers: time [s], x, y, yaw [rad]
        self.est_writer.writerow(['t', 'x', 'y', 'yaw'])
        self.gt_writer.writerow(['t', 'x', 'y', 'yaw'])

        # Subscriptions
        self.est_sub = self.create_subscription(
            Odometry,
            '/odom_est',
            self.odom_est_callback,
            10
        )

        self.gt_sub = self.create_subscription(
            Odometry,
            '/odom_sim',
            self.odom_gt_callback,
            10
        )

        self.get_logger().info('OdomRecorder started, writing odom_est.csv and odom_sim.csv')

    def odom_est_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.est_writer.writerow([t, x, y, yaw])

    def odom_gt_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.gt_writer.writerow([t, x, y, yaw])

    def destroy_node(self):
        self.est_file.close()
        self.gt_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
