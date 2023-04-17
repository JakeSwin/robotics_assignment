#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class DriveForward(Node):

    def __init__(self):
        super().__init__('forward')
        self.publisher = self.create_publisher(Twist, '/forward_vel', 1)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.35
        self.publisher.publish(msg)

def main(args=None):
    print('Starting drive_forward.py')
    rclpy.init(args=args)

    drive_forward = DriveForward()

    while rclpy.ok():
        rclpy.spin_once(drive_forward)

    drive_forward.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()