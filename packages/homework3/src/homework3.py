#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Homework3:
    def __init__(self):
        rospy.Subscriber("lol_new", Float32, self.callback)
        self.pub = rospy.Publisher("lol_new1", Float32, queue_size=10)
        self.x = 0

    def callback(self, data):
        self.x += data.data
        self.pub.publish(self.x)

if __name__ == '__main__':
    rospy.init_node('homework3')
    Homework3()

    rospy.spin()
