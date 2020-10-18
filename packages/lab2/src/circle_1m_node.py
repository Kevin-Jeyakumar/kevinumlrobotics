#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Circle:
    def __init__(self):
        #rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("/duckduckgone/car_cmd_switch_node/cmd", duckietown_msgs, queue_size=10)

    def circ(self,y,x):
        circ_msg=Twist2DStamped()
        circ_msg.v=y
        circ_msg.omega=x

        self.pub.publish(circ_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('circle_1m_node')
        ob=Circle()
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            ob.circ(0.4,2)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass



