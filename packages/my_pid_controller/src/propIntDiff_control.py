#!/usr/bin/env python3

import rospy
from std_msgs import Float32
from propIntDiff.py import PID

class PID_Control:
    def __init__(self):
        self.pub = rospy.Publish("control_input", Float32, queue_size=10)
        rospy.Subscriber("error", Float32, self.callback)
        timing=rospy.get_rostime()
        self.obj=PID(1,1,1,timing.secs)
    
    def callback(self, data):
        er=data.data
        timing=rospy.get_rostime()
        inp=self.obj.calculateSignal(er, timing.secs)
        self.pub.publish(inp)


if __name__ == "__main__" :
    rospy.init_node('pid_control')
    PID_Control()
    rospy.spin()
