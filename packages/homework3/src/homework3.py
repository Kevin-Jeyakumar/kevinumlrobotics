#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Homework3:
    def __init__(self):
        rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("changed_total", Float32, queue_size=10)
        self.x = 0

    def callback(self, data):
        self.pub.publish(data.data)

if __name__ == '__main__':
    rospy.init_node('homework3')
    Homework3()

    rospy.spin()
