#!/usr/bin/env python

import cv2
import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

from math import sin, cos
from cv_bridge import CvBridge

class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser_nav')
        self.colour_list = [
            {"name": "yellow", "min_hsv": (25, 150, 50), "max_hsv": (40, 255, 255)},
            {"name": "red", "min_hsv": (0, 150, 50), "max_hsv": (7, 255, 255)},
            {"name": "blue", "min_hsv": (80, 150, 50), "max_hsv": (120, 255, 255)},
            {"name": "green", "min_hsv": (40, 150, 50), "max_hsv": (75, 255, 255)},
        ]
        self.current_colour = None
        self.turn = 0.0
        self.current_pose_stamped = PoseStamped()
        self.current_scan = LaserScan()

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 1)

        self.br = CvBridge()

    def camera_callback(self, data):
        cv2.namedWindow("Image window", 1)

        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        current_frame_small = cv2.resize(current_frame, (0,0), fx=0.4, fy=0.4)
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        if self.current_colour == None:
            for colour in self.colour_list:
                current_frame_mask = cv2.inRange(current_frame_hsv, colour["min_hsv"], colour["max_hsv"]) 
                contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
                if len(contours) > 0:
                    M = cv2.moments(contours[0])
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        if M['m00'] > 40000 and cy < 500:
                            self.current_colour = colour
                            distance_from_center = (1920 / 2) - cx
                            degrees = distance_from_center * 0.153994
                            print(f"Distance from center: {distance_from_center}")
                            print(f"Degrees: {degrees}")
                            print(f"Scan angle min: {self.current_scan.angle_min}")
                            print(f"Scan angle max: {self.current_scan.angle_max}")
                            print(f"Scan angle inc: {self.current_scan.angle_increment}")
                            if degrees > 0:
                                print("Degrees greater than 0")
                                distance = self.current_scan.ranges[int(degrees)] - 1
                                x_dist = cos(degrees) * distance
                                y_dist = sin(degrees) * distance
                                goal = self.current_pose_stamped
                                goal.pose.position.x += x_dist
                                goal.pose.position.x += y_dist
                                self.goal_publisher.publish(goal)
        else:
            current_frame_mask = cv2.inRange(current_frame_hsv, self.current_colour["min_hsv"], self.current_colour["max_hsv"]) 

            contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # Sort by area (keep only the biggest one)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
            # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
            current_frame_contours = cv2.drawContours(current_frame, contours, 0, (0, 255, 0), 10)
        
            if len(contours) > 0:
                # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                M = cv2.moments(contours[0]) # only select the largest controur
                if M['m00'] > 0:
                    # find the centroid of the contour
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    # print(f"Contour Area: {M['m00']}")
                    #print("Centroid of the biggest area: ({}, {})".format(cx, cy))

                    # Draw a circle centered at centroid coordinates
                    # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                    cv2.circle(current_frame, (round(cx), round(cy)), 20, (255, 255, 255), -1)
                            
                    # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920

                    distance_from_center = (1920 / 2) - cx
                    print(f"Distance from center: {distance_from_center}")
                    
            else:
                print("No Centroid Found")
                self.current_colour = None
            current_frame_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4)

        cv2.imshow("Image window", current_frame_small)
        cv2.waitKey(1)

    def pose_callback(self, msg):
        current_pos = PoseStamped()
        current_pos.header.frame_id = msg.header.frame_id
        current_pos.pose.position = msg.pose.pose.position
        current_pos.pose.orientation.w = 1.0
        self.current_pose_stamped = current_pos
        print(f"x: {current_pos.pose.position.x}, y: {current_pos.pose.position.y}")
    
    def scan_callback(self, msg):
        self.current_scan.angle_min = msg.angle_min
        self.current_scan.angle_max = msg.angle_max
        self.current_scan.angle_increment = msg.angle_increment
        self.current_scan.ranges = msg.ranges

def main(args=None):
    print('Starting colour_chaser.py')

    rclpy.init(args=args)
    
    colour_chaser = ColourChaser()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()