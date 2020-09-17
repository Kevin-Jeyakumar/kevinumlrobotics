#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher("/homework1/delta", Float32, queue_size=10)
        self.x=0

    def talk(self):
        self.pub.publish(x)

if __name__ == '__main__':
    try:
        t=Talker()
        rospy.init_node('talker')
        rate=rospy.Rate(0.1) # 0.1Hz
        while not rospy.is_shutdown():
            t.talk()
            t.x+=2
            rate.sleep()
    except rospy.ROSInterruptException:
        pass 

