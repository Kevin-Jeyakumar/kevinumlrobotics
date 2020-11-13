#!/usr/bin/env python3

import rospy
import numpy as np
from odometry_hw.msg import *

class Homework7:
    def __init__(self):
        rospy.Subscriber("/dist_wheel", DistWheel, self.callback)
        self.pub = rospy.Publisher("/pose", Pose2D, queue_size=10)
        self.pos = Pose2D() 
        self.pos.x = 0
        self.pos.y = 0
        self.pos.theta = 0
        self.pub.publish(self.pos)

    def callback(self,data):
        del_s = data.dist_wheel_left + data.dist_wheel_right
        del_s /= 2
        del_theta = data.dist_wheel_right - data.dist_wheel_left
        del_theta /= 0.1

        self.pos.x += del_s * np.cos(self.pos.theta + (del_theta/2))
        self.pos.y += del_s * np.sin(self.pos.theta + (del_theta/2))
        self.pos.theta += del_theta

        self.pub.publish(self.pos)

if __name__ == "__main__":
    rospy.init_node("homework7_node")
    Homework7()
    
    rospy.spin()

