#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')
        self.publisher = self.create_publisher(Twist, '/wander_vel', 1)
        self.recovery_twist = self.create_publisher(Twist, '/recovery_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rotate = False
        self.turn = 0.0

    def listener_callback(self, msg):
        # self.get_logger().info(f"Scan range min: {msg.range_min}, max: {msg.range_max}")
        # self.get_logger().info(f"Scan ranges count: {len(msg.ranges[90:270])}")
        min_front = min(msg.ranges[0:60] + msg.ranges[300:360])
        avg_left = sum(msg.ranges[85:95]) / 10
        avg_right = sum(msg.ranges[265:275]) / 10

        self.get_logger().info(f"avg_left: {avg_left}, avg_right: {avg_right}")
        self.get_logger().info(f"Min scan: {min_front}\n")
        if min_front < 0.70:
            if not self.rotate:
                self.rotate = True
                #self.turn = 0.2 if min(msg.ranges[85:95]) > min(msg.ranges[265:275]) else -0.2
                self.turn = 0.2 if avg_left > avg_right else -0.2
        elif avg_left < 2.5 and avg_left > 0.8 and avg_right < 2.5 and avg_right > 0.8:
            #diff = avg_right - avg_left
            diff = avg_left - avg_right
            self.turn = (diff / 1.7) * 0.2
            self.get_logger().info(f"Turning: {self.turn}\n")
        else:
            self.turn = 0.0
            self.rotate = False

    def timer_callback(self):
        if not self.rotate: 
            msg = Twist()
            msg.linear.x = 0.35
            msg.angular.z = self.turn
            self.publisher.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg.linear)
        else:
            msg = Twist()
            msg.angular.z = self.turn
            self.recovery_twist.publish(msg)
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