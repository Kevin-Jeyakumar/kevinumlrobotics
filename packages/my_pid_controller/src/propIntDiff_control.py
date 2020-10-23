#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import propIntDiff

class PID_Control(propIntDiff):
    def __init__(self,p,i,d,time):
        propIntDiff.__init__(p,i,d,time)
        self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
        rospy.Subscriber("error", Float32, self.callback)
        timing=rospy.get_rostime()
        #obj=PID(1,1,1,timing.secs)
        self.ob=obj
    
    def callback(self, data):
        er=data.data
        timing=rospy.get_rostime()
        inp=self.calculateSignal(er, timing.secs)
        self.pub.publish(inp)


if __name__ == "__main__" :
    rospy.init_node('pid_control')
    rospy.set_param("controller_ready","true")
    timing=rospy.get_rostime()
    PID_control(1,1,1,timing.secs)
    #PID_Control()
    rospy.spin()
