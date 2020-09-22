#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Homework3:
    def __init__(self):
        rospy.Subscriber("/homework1/total", Float32, self.callback)
        self.pub = rospy.Publisher("converted_total", Float32, queue_size=10)
        self.x = 0 
        self.unit = "meters"

    def callback(self, data):
        if rospy.has_param("unit"):
            self.unit = rospy.get_param("unit")
        else self.unit = "meters"
        if(self.unit == "meters")
            self.x=data.data*0.3048
        else if(self.unit == "smoots")
            self.x = data.data/5.5833
        else if(self.unit == "feet")
            self.x = data.data
        self.pub.publish(self.x)

        temp_string = "Unit: %s; Input: %f feet; Output: %f %s" % (self.unit, data.data, self.x, self.unit)
        rospy.loginfo(temp_string)

if __name__ == '__main__':
    rospy.init_node('homework3')
    Homework3()

    rospy.spin()
