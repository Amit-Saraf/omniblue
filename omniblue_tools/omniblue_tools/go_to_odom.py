#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import sys

class GoToOdom(Node):

    def __init__(self, x, y, yaw):
        super().__init__('go_to_odom')

        self.goal_x = x
        self.goal_y = y
        self.goal_yaw = yaw

        self.cmd_pub = self.create_publisher(Twist, '/omniblue/cmd_vel', 10)
        self.create_subscription(Odometry, '/omniblue/odom', self.odom_cb, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def control_loop(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)

        cmd = Twist()

        if dist > 0.05:
            cmd.linear.x = 1.0 * dx
            cmd.linear.y = 1.0 * dy
        else:
            yaw_error = self.goal_yaw - self.yaw
            if abs(yaw_error) > 0.05:
                cmd.angular.z = 1.5 * yaw_error
            else:
                self.get_logger().info("GOAL REACHED")
                self.cmd_pub.publish(Twist())
                rclpy.shutdown()
                return

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GoToOdom(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    rclpy.spin(node)

if __name__ == '__main__':
    main()
