#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench


class PositionController(Node):

    def __init__(self):
        super().__init__('position_controller')

        # ============================
        # Vehicle Parameters
        # ============================

        self.mass = 0.027
        self.g = 9.81

        # ============================
        # Controller Gains
        # ============================

        self.kp = np.array([1.0, 1.0, 3.0])
        self.kd = np.array([0.5, 0.5, 1.5])

        # ============================
        # Desired State
        # ============================

        self.pd = np.array([0.0, 0.0, 1.0])
        self.vd = np.array([0.0, 0.0, 0.0])
        self.ad = np.array([0.0, 0.0, 0.0])

        # Desired yaw (future use)
        self.psi_d = 0.0

        # ============================
        # Current State
        # ============================

        self.p = np.zeros(3)
        self.v = np.zeros(3)

        self.odom_received = False

        # ============================
        # Subscriber
        # ============================

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            10
        )

        # ============================
        # Publisher
        # ============================

        self.force_pub = self.create_publisher(
            Wrench,
            '/block/force',
            10
        )

        # ============================
        # Control Loop
        # ============================

        self.timer = self.create_timer(
            0.01,
            self.control_loop
        )

        self.get_logger().info(
            'Position Controller Started'
        )

    def odom_callback(self, msg):

        self.p[0] = msg.pose.pose.position.x
        self.p[1] = msg.pose.pose.position.y
        self.p[2] = msg.pose.pose.position.z

        self.v[0] = msg.twist.twist.linear.x
        self.v[1] = msg.twist.twist.linear.y
        self.v[2] = msg.twist.twist.linear.z

        self.odom_received = True

    def control_loop(self):

        if not self.odom_received:
            return

        # Position Error
        ep = self.pd - self.p

        # Velocity Error
        ev = self.vd - self.v

        # PD + Feedforward + Gravity Compensation
        F = (
            self.kp * ep +
            self.kd * ev +
            self.mass * self.ad +
            np.array([0.0, 0.0, self.mass * self.g])
        )

        # Saturation
        F[0] = np.clip(F[0], -5.0, 5.0)
        F[1] = np.clip(F[1], -5.0, 5.0)
        F[2] = np.clip(F[2], 0.0, 5.0)

        msg = Wrench()

        msg.force.x = float(F[0])
        msg.force.y = float(F[1])
        msg.force.z = float(F[2])

        # No attitude control yet
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0

        self.force_pub.publish(msg)

    def setpoint(self, x, y, z):

        self.pd = np.array([x, y, z])


def main(args=None):

    rclpy.init(args=args)

    node = PositionController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()