#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Listener:
    def __init__(self):
        rospy.Subscriber("/homework1/total", Float32, self.callback)

    def callback(self, data):
        rospy.loginfo(data.data) 


if __name__ == '__main__':
    rospy.init_node('listener')
    Listener()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

