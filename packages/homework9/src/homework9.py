#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class Homework9:
    def __init__(self):
        rospy.Subscriber("image_cropped", Image, self.notuncanny)
        rospy.Subscriver("image_white", Image, self.white_fn)
        rospy.Subscriber("image_yellow", Image, self.yellow_fn)
        self.pub = rospy.Publisher("image_lines_white", Image, queue_size=10)
        self.pub1 = rospy.Publisher("image_lines_yellow", Image, queue_size=10)
        self.pub2 = rospy.Publisher("image_lines_all", Image, queue_size=10)
        self.bridge = CvBridge()

