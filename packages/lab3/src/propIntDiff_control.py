#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from propIntDiff import PID

class PID_Control(PID):
    def __init__(self,p,i,d,time):
        PID.__init__(self,p,i,d,time)
        self.pub = rospy.Publisher("control_input", Float32, queue_size=10)
        rospy.Subscriber("error", Float32, self.callback)
        timing=rospy.get_rostime()
    
    def callback(self, data):
        er=data.data
        timing=rospy.get_rostime()
        time_tmp=timing.secs+(timing.nsecs/1000000000)
        temp="time ",(self.past_time_stamp-time_tmp)
        inp=self.calculateSignal(er, time_tmp)
        rospy.loginfo(temp)
        self.pub.publish(inp)


if __name__ == "__main__" :
    rospy.init_node('pid_control')
    rospy.set_param("controller_ready","true")
    timing=rospy.get_rostime()
    PID_Control(0.185,0.00009,1.9,timing.secs)#p=0.18,i=0.00009,d=1.9
    while rospy.get_param("controller_ready")=="true":
        pass
    
    #rospy.spin()

