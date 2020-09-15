#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher("/delta", Float32, queue_size=10)

if __name__ == '__main__':
    rospy.init_node('talker')
    Talker()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

