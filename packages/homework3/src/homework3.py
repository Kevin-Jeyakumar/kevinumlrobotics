#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Homework3:
    def __init__(self):
        rospy.Subscriber("hw3_temp", Float32, self.callback)
        self.pub = rospy.Publisher("converted_total", Float32, queue_size=10)
        self.x = 0 
        self.current_unit = "feet"
        self.required_unit = "meters"

    def callback(self, data):
        if rospy.has_param("unit"):
            self.unit = rospy.get_param("unit")
        #if(self.unit == "meters")
        #    self.xt
        self.pub.publish(data.data)

if __name__ == '__main__':
    rospy.init_node('homework3')
    Homework3()

    rospy.spin()
