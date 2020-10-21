#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Circle:
    def __init__(self):
        #rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)

    def move(self,y,x):
        move_msg=Twist2DStamped()
        move_msg.v=y
        move_msg.omega=x

        self.pub.publish(move_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('circle_1m_node')
        ob=Circle()
        rate=rospy.Rate(10)
        for count in range(0,100):#10 second timer
            ob.move(0.7,1.5)
            rate.sleep()
        ob.move(0,0)
    except rospy.ROSInterruptException:
        pass



