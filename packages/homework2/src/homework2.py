#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Homework2:
    def __init__(self):
        rospy.Subscriber("my_delta", Float32, self.callback)
        self.pub = rospy.Publisher("my_total", Float32, queue_size=10)
        self.my_total = 0

    def callback(self, data):
        self.my_total = data.data
        self.pub.publish(self.my_total)

if __name__ == '__main__':
    rospy.init_node('homework2')
    Homework2()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

