#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from duckietown_msgs.msg import Segment, SegmentList 
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class Lab5:
    def __init__(self):
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher("line_detector_node/segment_list", SegmentList, queue_size=1)
        self.pub0 = rospy.Publisher("/image_cropped", Image, queue_size=1)
        self.pub1 = rospy.Publisher("/image_white", Image, queue_size=1)
        self.pub2 = rospy.Publisher("/image_yellow", Image, queue_size=1)
        self.pub3 = rospy.Publisher("/image_white_filtered", Image, queue_size=1)
        self.pub4 = rospy.Publisher("/image_yellow_filtered", Image, queue_size=1)
        self.pub45 = rospy.Publisher("/image_all_filtered", Image, queue_size=1)
        self.pub5 = rospy.Publisher("/image_lines_white", Image, queue_size=1)
        self.pub6 = rospy.Publisher("/image_lines_yellow", Image, queue_size=1)
        self.pub7 = rospy.Publisher("/image_lines_all", Image, queue_size=1)
        self.pub8 = rospy.Publisher("/debug_canny", Image, queue_size=1)
        self.pub9 = rospy.Publisher("/debug_canny_white", Image, queue_size=1)
        self.pub10 = rospy.Publisher("/debug_canny_yellow", Image, queue_size=1)
        self.bridge = CvBridge()
        self.count = 0

    #-----Drawing_Lines(For_Debugging)-----#
    def output_lines1(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255, 0, 0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0], l[1]), 2, (0, 255, 0))
                cv2.circle(output, (l[2], l[3]), 2, (0, 0, 255))
        return output

    #-----Creating_SegmentList-----#
    def output_lines(self, seg_list, lines, color): 
        #seg_list = SegmentList()
        image_size = (120, 160) #(160, 120)
        arr_cutoff = np.array([0, 40, 0, 40])
        arr_ratio = np.array([1./image_size[1], 1./image_size[0], 1./image_size[1], 1./image_size[0]])
        if lines is not None:
            for i in range(len(lines)):
                l = (lines[i][0]+arr_cutoff)*arr_ratio
                segment = Segment()
                if color == 0:
                    segment.color = segment.WHITE
                elif color == 1:
                    segment.color = segment.YELLOW
                segment.pixels_normalized[0].x = l[0]
                segment.pixels_normalized[0].y = l[1]
                #segment.points[0].z = 0
                segment.pixels_normalized[1].x = l[2]
                segment.pixels_normalized[1].y = l[3]
                #segment.points[1].z = 0
                seg_list.segments.append(segment)
            #self.pub.publish(seg_list)
        return seg_list

    def callback(self, data):
        if self.count >= 50:
            rospy.loginfo("KEVIN JEYAKUMAR LANE FOLLOWING CODE")
            self.count = 0
        self.count += 1
        cv_img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

        #-----Resizing-and-Cropping-----#
        image_size = (160, 120)
        cv_img2 = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        y_size = 40 #len(cv_img[:,0,0])
        cv_cropped = cv_img2[y_size:,:,:]
        ros_cropped = self.bridge.cv2_to_imgmsg(cv_cropped, "bgr8")
        self.pub0.publish(ros_cropped)

        #-----Kernel-----#
        cv_hsv = cv2.cvtColor(cv_cropped, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))

        #-----White_Mask-----#
        cv_white_filter = cv2.inRange(cv_hsv, (1,0,100), (180,50,255))
        cv_eroded_white_filter = cv2.erode(cv_white_filter, kernel)
        cv_white_mask = cv2.dilate(cv_eroded_white_filter, kernel2)
        ros_white = self.bridge.cv2_to_imgmsg(cv_white_mask, "mono8")
        self.pub1.publish(ros_white)
        cv_white_filtered_image = cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_white_mask)
        ros_white_filtered = self.bridge.cv2_to_imgmsg(cv_white_filtered_image, "bgr8")
        self.pub3.publish(ros_white_filtered)

        #-----Yellow_Mask-----#
        cv_yellow_filter = cv2.inRange(cv_hsv, (25,100,100), (40, 255, 255))
        cv_eroded_yellow_filter = cv2.erode(cv_yellow_filter, kernel)
        cv_yellow_mask = cv2.dilate(cv_eroded_yellow_filter, kernel2)
        ros_yellow = self.bridge.cv2_to_imgmsg(cv_yellow_mask, "mono8")
        self.pub2.publish(ros_yellow)
        cv_yellow_filtered_image = cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_yellow_mask)
        ros_yellow_filtered = self.bridge.cv2_to_imgmsg(cv_yellow_filtered_image, "bgr8")
        self.pub4.publish(ros_yellow_filtered)

        #-----White_and_Yellow_Mask-----#
        cv_all_mask = cv2.bitwise_and(cv_yellow_mask, cv_white_mask)
        cv_all_filtered_image = cv2.bitwise_and(cv_cropped, cv_cropped, mask=cv_all_mask)
        ros_all_filtered = cv2.bridge.cv2_to_imgmsg(cv_all_filtered_image, "bgr8")
        self.pub45.publish(ros_all_filtered)

        #-----Canny_Edges-----#
        cv_canny = cv2.Canny(cv_cropped, 254, 255)
        ros_canny = self.bridge.cv2_to_imgmsg(cv_canny, "mono8")
        self.pub8.publish(ros_canny)

        #-----White_Lines-----#
        cv_can_white = cv2.bitwise_and(cv_canny, cv_white_mask)
        ros_can_white = self.bridge.cv2_to_imgmsg(cv_can_white, "mono8")
        self.pub9.publish(ros_can_white)
        white_hough_lines = cv2.HoughLinesP(cv_can_white, 1, (np.pi/180), 10, minLineLength = 5, maxLineGap = 3)
        segment_list = SegmentList()
        segment_list = self.output_lines(segment_list, white_hough_lines, 0)
        cv_hough = self.output_lines1(cv_cropped, white_hough_lines)
        ros_white_lines = self.bridge.cv2_to_imgmsg(cv_hough, "bgr8")
        self.pub5.publish(ros_white_lines)

        #-----Yellow_Lines-----#
        cv_can_yellow = cv2.bitwise_and(cv_canny, cv_yellow_mask)
        ros_can_yellow = self.bridge.cv2_to_imgmsg(cv_can_yellow, "mono8")
        self.pub10.publish(ros_can_yellow)
        yellow_hough_lines = cv2.HoughLinesP(cv_can_yellow, 1, (np.pi/180), 10, minLineLength = 5, maxLineGap = 3)
        segment_list = self.output_lines(segment_list, yellow_hough_lines, 1)
        cv_hough = self.output_lines1(cv_cropped, yellow_hough_lines)
        ros_yellow_lines = self.bridge.cv2_to_imgmsg(cv_hough, "bgr8")
        self.pub6.publish(ros_yellow_lines)
        self.pub.publish(segment_list)

        #-----All_Lines-----#
        cv_hough = self.output_lines1(cv_hough, white_hough_lines)
        ros_all_lines = self.bridge.cv2_to_imgmsg(cv_hough, "bgr8")
        self.pub7.publish(ros_all_lines)

        
if __name__ == "__main__":
    rospy.init_node("lab5_node")
    Lab5()
    rospy.spin()
