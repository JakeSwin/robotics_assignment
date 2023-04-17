#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Recovery(Node):

    def __init__(self):
        super().__init__('recovery')
        self.publisher = self.create_publisher(Twist, '/recovery_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            1)
        self.subscription
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rotate = False
        self.turn = 0.0

    def listener_callback(self, msg):
        min_front = min(msg.ranges[0:60] + msg.ranges[300:360])
        avg_left = sum(msg.ranges[85:95]) / 10
        avg_right = sum(msg.ranges[265:275]) / 10

        if min_front < 0.70:
            if not self.rotate:
                self.rotate = True
                self.turn = 0.2 if avg_left > avg_right else -0.2
        else:
            self.turn = 0.0
            self.rotate = False

    def timer_callback(self):
        if self.rotate:
            msg = Twist()
            msg.angular.z = self.turn
            self.publisher.publish(msg)

def main(args=None):
    print('Starting recovery.py')
    rclpy.init(args=args)

    recovery = Recovery()

    while rclpy.ok():
        rclpy.spin_once(recovery)

    recovery.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()