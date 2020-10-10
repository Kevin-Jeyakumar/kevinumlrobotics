#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from homework4.msg import homework4_msgs

class Homework4:
    def __init__(self):
        rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("converted_total", homework4_msgs, queue_size=10)
        self.x = 0 
        self.unit = "meters"

    def callback(self, data):
        if rospy.has_param("unit"):
            self.unit = rospy.get_param("unit")
        else:
            self.unit = "meters"
        if self.unit == "meters" :
            self.x=data.data*0.3048
        elif self.unit == "smoots" :
            self.x = data.data/5.5833
        elif self.unit == "feet" :
            self.x = data.data

        hw4_msg=homework4_msgs()
        hw4_msg.num=self.x
        hw4_msg.units=self.unit

        self.pub.publish(hw4_msg)


if __name__ == '__main__':
    rospy.init_node('homework4_node')
    Homework4()


    rospy.spin()
