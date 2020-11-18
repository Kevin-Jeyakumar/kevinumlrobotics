#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String 
from duckietown_msgs.msg import WheelsCmdStamped

class Lab4:
    def __init__(self):
        rospy.Subscriber("wheels_driver_node/wheels_cmd", WheelsCmdStamped, self.callback)
        self.pub = rospy.Publisher("/lab4_output", String, queue_size=10)
        #self.pos = Pose2D()
        self.x = 0
        self.y = 0
        self.theta = 0
        #self.x_list = list()
        #self.y_list = list()
        #self.x_list.append(self.x)
        #self.y_list.append(self.y)
        my_time = rospy.get_rostime()
        self.past_time = my_time.secs + (my_time.nsecs/1000000000)
        rospy.loginfo("-----ODOMETER STARTED-----")

    def callback(self,data):
        my_time = rospy.get_rostime()
        present_time = my_time.secs + (my_time.nsecs/1000000000)
        del_t = present_time - self.past_time
        dist_left = data.vel_left * del_t
        dist_right = data.vel_right * del_t
        del_s = dist_left + dist_right
        del_s /= 2
        del_theta = dist_right - dist_left
        del_theta /= 0.1

        self.x += del_s * np.cos(self.theta + (del_theta/2))
        self.y += del_s * np.sin(self.theta + (del_theta/2))
        self.theta += del_theta
        self.past_time = present_time

        #self.x_list.append(self.x)
        #self.y_list.append(self.y)

        #mylogstr = "x:",self.x,"y:",self.y
        my_angle = self.theta*360/(2*np.pi)
        if my_angle>360 or my_angle<-360:
            my_angle = my_angle%360
        #rospy.loginfo("x: {:.3f}, y: {:.3f}, theta: {:.3f}".format(self.x,self.y,my_angle))
        output_str = "x: {:.3f}, y: {:.3f}, theta: {:.3f}".format(self.x,self.y,my_angle)
        self.pub.publish(output_str)

if __name__ == "__main__":
    rospy.init_node("lab4_node")
    Lab4()
    
    rospy.spin()

