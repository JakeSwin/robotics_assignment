#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')
        self.colour_list = [
            {"name": "yellow", "min_hsv": (25, 150, 50), "max_hsv": (40, 255, 255)},
            {"name": "red", "min_hsv": (0, 150, 50), "max_hsv": (7, 255, 255)},
            {"name": "blue", "min_hsv": (80, 150, 50), "max_hsv": (120, 255, 255)},
            {"name": "green", "min_hsv": (40, 150, 50), "max_hsv": (75, 255, 255)},
        ]
        self.current_colour = None
        self.rotate = True
        self.turn = 0.15

        self.publisher = self.create_publisher(Twist, '/colour_vel', 1)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 1)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.br = CvBridge()
        self.current_frame = ""

    def camera_callback(self, data):
        cv2.namedWindow("Image window", 1)

        self.current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        current_frame_small = cv2.resize(self.current_frame, (0,0), fx=0.4, fy=0.4)
        current_frame_hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)

        if self.current_colour == None:
            self.rotate = True
            for colour in self.colour_list:
                current_frame_mask = cv2.inRange(current_frame_hsv, colour["min_hsv"], colour["max_hsv"]) 
                contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
                if len(contours) > 0:
                    M = cv2.moments(contours[0])
                    if M['m00'] > 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        if M['m00'] > 30000:
                            self.current_colour = colour

        else:
            current_frame_mask = cv2.inRange(current_frame_hsv, self.current_colour["min_hsv"], self.current_colour["max_hsv"]) 

            contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # Sort by area (keep only the biggest one)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
            # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
            current_frame_contours = cv2.drawContours(self.current_frame, contours, 0, (0, 255, 0), 10)
        
            if len(contours) > 0:
                # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
                M = cv2.moments(contours[0]) # only select the largest controur
                if M['m00'] > 0:
                    self.get_logger().info(f"Contour Size: {M['m00']}", throttle_duration_sec=1)
                    # find the centroid of the contour
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.get_logger().info(f"cy: {cy}", throttle_duration_sec=1)
                    #print("Centroid of the biggest area: ({}, {})".format(cx, cy))

                    # Draw a circle centered at centroid coordinates
                    # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                    cv2.circle(self.current_frame, (round(cx), round(cy)), 20, (255, 255, 255), -1)
                            
                    # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920

                    # if center of object is to the left of image center move left
                    if cx < 900:
                        self.rotate = True
                        self.turn = 0.1
                    elif cx >= 1200:
                        self.rotate = True
                        self.turn = -0.1
                    else:
                        self.rotate = False

                    if M['m00'] > 150000:
                        self.get_logger().info(f"found: {self.current_colour['name']} with contour size: {M['m00']}")
                        self.colour_list.remove(self.current_colour)
                        self.current_colour = None
                    
            else:
                print("No Centroid Found")
                self.current_colour = None
            current_frame_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4)

        cv2.imshow("Image window", current_frame_small)
        cv2.waitKey(1)

    def timer_callback(self):
        if self.rotate: 
            msg = Twist()
            msg.angular.z = self.turn
            self.publisher.publish(msg)

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