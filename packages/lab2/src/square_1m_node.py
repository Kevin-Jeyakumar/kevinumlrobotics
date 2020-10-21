#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Square:
    def __init__(self):
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

    def move(self,y,x):
        move_msg=Twist2DStamped()
        move_msg.v=y
        move_msg.omega=x

        self.pub.publish(move_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('square_1m_node')
        ob=Square()
        rate=rospy.Rate(10)
        count=0
        for c in range(0,4):#one iteration for each side of the square
            for count in range(0,10):
                ob.move(0,0)#stop
                rate.sleep()
            for count in range(0,20):
                ob.move(0.5,0)#forward 1meter
                rate.sleep()
            for count in range(0,10):
                ob.move(0,0)#stop
                rate.sleep()
            for count in range(0,6):
                ob.move(0,3)#90degree left turn
                rate.sleep()
            ob.move(0,0)
        ob.move(0,0)
    except rospy.ROSInterruptException:
        pass



