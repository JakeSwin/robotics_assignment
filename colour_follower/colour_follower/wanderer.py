#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rotate = False
        self.turn_left = True

    def listener_callback(self, msg):
        # self.get_logger().info(f"Scan range min: {msg.range_min}, max: {msg.range_max}")
        # self.get_logger().info(f"Scan ranges count: {len(msg.ranges[90:270])}")
        min_scan = min(msg.ranges[0:70] + msg.ranges[290:360])

        self.get_logger().info(f"Min scan: {min_scan}")
        if min_scan < 0.8:
            self.rotate = True
        else:
            self.rotate = False

    def timer_callback(self):
        if not self.rotate: 
            msg = Twist()
            msg.linear.x = 0.2
            self.publisher.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg.linear)
        else:
            msg = Twist()
            msg.angular.z = 0.2
            self.publisher.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg.linear)


def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()

    while rclpy.ok():
        rclpy.spin_once(wanderer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()