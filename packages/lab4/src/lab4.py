#!/usr/bin/env python3

import rospy
import numpy as np
from duckiebot_msgs.msg import *

class Lab4:
    def __init__(self):
        rospy.Subscriber("wheels_driver_node/wheeld_cmd", DistWheel, self.callback)
        #self.pub = rospy.Publisher("/pose", Pose2D, queue_size=10)
        #self.pos = Pose2D()
        self.x = 0
        self.y = 0
        self.theta = 0
        self.x_list = list()
        self.y_list = list()
        self.x_list.append(self.x)
        self.y_list.append(self.y)
        my_time = rospy.get_rostime()
        self.past_time = my_time.secs + (my_time.nsecs/1000000000)

    def callback(self,data):
        my_time = rospy.get_rostime()
        del_t = my_time.secs + (my_time.nsecs/1000000000)
        del_t = self.past_time - del_t
        del_s = data.dist_wheel_left + data.dist_wheel_right
        del_s /= 2
        del_theta = data.dist_wheel_right - data.dist_wheel_left
        del_theta /= 0.1

        self.x += del_s * np.cos(self.pos.theta + (del_theta/2))
        self.y += del_s * np.sin(self.pos.theta + (del_theta/2))
        self.theta += del_theta

        self.x_list.append(self.x)
        self.y_list.append(self.y)

if __name__ == "__main__":
    rospy.init_node("lab4_node")
    Lab4()
    
    rospy.spin()

