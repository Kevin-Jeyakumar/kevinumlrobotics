#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Homework9:
    def __init__(self):
        rospy.Subscriber("image_cropped", Image, self.notuncanny)
        rospy.Subscriber("image_white", Image, self.white_fn)
        rospy.Subscriber("image_yellow", Image, self.yellow_fn)
        self.pub = rospy.Publisher("image_lines_white", Image, queue_size=10)
        self.pub1 = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.pub2 = rospy.Publisher("image_lines_all", Image, queue_size=10)
        self.pub3 = rospy.Publisher("debug_canny", Image, queue_size=10)
        self.pub4 = rospy.Publisher("debug_canny_white", Image, queue_size=10)
        self.pub5 = rospy.Publisher("debug_canny_yellow", Image, queue_size=10)
        self.bridge = CvBridge()
        self.cv_canny = None
        self.cv_cropped = None
        self.white_hough_lines = None
        self.yellow_hough_lines = None

    def notuncanny(self,data):
        self.cv_cropped = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.cv_canny = cv2.Canny(self.cv_cropped, 254, 255)
        ros_canny = self.bridge.cv2_to_imgmsg(self.cv_canny, "mono8")
        self.pub3.publish(ros_canny)

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    def white_fn(self,data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "mono8")
        if self.cv_canny is not None and self.cv_cropped is not None:
            cv_can_white = cv2.bitwise_and(self.cv_canny, cv_img)
            ros_can_white = self.bridge.cv2_to_imgmsg(cv_can_white, "mono8")
            self.pub4.publish(ros_can_white)
            self.white_hough_lines = cv2.HoughLinesP(cv_can_white, 1, (np.pi/180), 10, minLineLength = 5, maxLineGap = 3)
            cv_hough = self.output_lines(self.cv_cropped, self.white_hough_lines)
            ros_white_lines = self.bridge.cv2_to_imgmsg(cv_hough, "bgr8")
            self.pub.publish(ros_white_lines)
    
    def yellow_fn(self,data):
        cv_img = self.bridge.imgmsg_to_cv2(data, "mono8")
        if self.cv_canny is not None and self.cv_cropped is not None:
            cv_can_yellow = cv2.bitwise_and(self.cv_canny, cv_img)
            ros_can_yellow = self.bridge.cv2_to_imgmsg(cv_can_yellow, "mono8")
            self.pub5.publish(ros_can_yellow)
            self.yellow_hough_lines = cv2.HoughLinesP(cv_can_yellow, 1, (np.pi/180), 10, minLineLength = 5, maxLineGap = 3)
            cv_hough = self.output_lines(self.cv_cropped, self.yellow_hough_lines)
            ros_yellow_lines = self.bridge.cv2_to_imgmsg(cv_hough, "bgr8")
            self.pub1.publish(ros_yellow_lines)
            if self.white_hough_lines is not None:
                cv_hough = self.output_lines(cv_hough, self.white_hough_lines)
                ros_all_lines = self.bridge.cv2_to_imgmsg(cv_hough, "bgr8")
                self.pub2.publish(ros_all_lines)


if __name__ == "__main__":
    rospy.init_node("homework9_node")
    Homework9()
    rospy.spin()
