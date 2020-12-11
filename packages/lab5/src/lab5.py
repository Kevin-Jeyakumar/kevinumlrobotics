#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import duckietown_msgs.msg import 
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class Lab5:
    def __init__(self):
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("/image_cropped", CompressedImage, queue_size=1)
        self.pub1 = rospy.Publisher("/image_white", CompressedImage, queue_size=1)
        self.pub2 = rospy.Publisher("/image_yellow", CompressedImage, queue_size=1)
        self.pub3 = rospy.Publisher("/image_white_filtered", CompressedImage, queue_size=1)
        self.pub4 = rospy.Publisher("/image_yellow_filtered", CompressedImage, queue_size=1)
        self.pub5 = rospy.Publisher("/image_lines_white", CompressedImage, queue_size=1)
        self.pub6 = rospy.Publisher("/image_lines_yellow", CompressedImage, queue_size=1)
        self.pub7 = rospy.Publisher("/image_lines_all", CompressedImage, queue_size=1)
        self.pub8 = rospy.Publisher("/debug_canny", CompressedImage, queue_size=1)
        self.pub9 = rospy.Publisher("/debug_canny_white", CompressedImage, queue_size=1)
        self.pub10 = rospy.Publisher("/debug_canny_yellow", CompressedImage, queue_size=1)
        self.bridge = CvBridge()

    #-----Drawing_Lines-----
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    def callback(self, data):
        cv_img = self.bridge.comressed_imgmsg_to_cv2(data, "bgr8")

        #-----Resizing-and-Cropping-----#
        image_size = (160, 120)
        cv_img2 = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        y_size = 40 #len(cv_img[:,0,0])
        cv_cropped = cv_img2[y_size:,:,:]
        ros_cropped = self.bridge.cv2_to_compressed_imgmsg(cv_cropped, "bgr8")
        #self.pub.publish(ros_cropped)

        #-----Kernel-----#
        cv_hsv = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11,11))

        #-----White_Mask-----#
        cv_white_filter = cv2.inRange(cv_hsv, (1,0,150), (180,30,255))
        cv_eroded_white_filter = cv2.erode(cv_white_filter, kernel)
        cv_white_mask = cv2.dilate(cv_eroded_white_filter, kernel2)
        ros_white = self.bridge.cv2_to_compressed_imgmsg(cv_white_mask, "mono8")
        #self.pub1.publish(ros_white)
        cv_white_filtered_image = cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_white_mask)
        ros_white_filtered = self.bridge.cv2_to_compressed_imgmsg(cv_white_filtered_image, "bgr8")
        #self.pub3.publish(ros_white_filtered)

        #-----Yellow_Mask-----#
        cv_yellow_filter = cv2.inRange(cv_hsv, (25,100,200), (40, 255, 255))
        cv_eroded_yellow_filter = cv2.erode(cv_yellow_filter, kernel)
        cv_yellow_mask = cv2.dilate(cv_eroded_yellow_filter, kernel2)
        ros_yellow = self.bridge.cv2_to_compressed_imgmsg(cv_yellow_mask, "mono8")
        #self.pub2.publish(ros_yellow)
        cv_yellow_filtered_image = cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_yellow_mask)
        ros_yellow_filtered = self.bridge.cv2_to_compressed_imgmsg(cv_yellow_filtered_image, "bgr8")
        #self.pub4.publish(ros_yellow_filtered)

        #-----Canny_Edges-----#
        cv_canny = cv2.Canny(cv_cropped, 254, 255)
        ros_canny = self.bridge.cv2_to_compressed_imgmsg(cv_canny, "mono8")
        #self.pub8.publish(ros_canny)

        #-----White_Lines-----#
        cv_can_white = cv2.bitwise_and(cv_canny, cv_white_mask)
        ros_can_white = self.bridge.cv2_to_compressed_imgmsg(cv_can_white, "mono8")
        #self.pub9.publish(ros_can_white)
        white_hough_lines = cv2.HoughLinesP(cv_can_white, 1, (np.pi/180), 10, minLineLength = 5, maxLineGap = 3)
        cv_hough = self.output_lines(cv_cropped, white_hough_lines)
        ros_white_lines = self.bridge.cv2_to_compressed_imgmsg(cv_hough, "bgr8")
        #self.pub5.publish(ros_white_lines)

        #-----Yellow_Lines-----#
        cv_can_yellow = cv2.bitwise_and(self.cv_canny, cv_yellow_mask)
        ros_can_yellow = self.bridge.cv2_to_compressed_imgmsg(cv_can_yellow, "mono8")
        #self.pub10.publish(ros_can_yellow)
        yellow_hough_lines = cv2.HoughLinesP(cv_can_yellow, 1, (np.pi/180), 10, minLineLength = 5, maxLineGap = 3)
        cv_hough = self.output_lines(self.cv_cropped, yellow_hough_lines)
        ros_yellow_lines = self.bridge.cv2_to_compressed_imgmsg(cv_hough, "bgr8")
        self.pub6.publish(ros_yellow_lines)

        
if __name__ == "__main__":
    rospy.init_node("lab5_node")
    Lab5()
    rospy.spin()
