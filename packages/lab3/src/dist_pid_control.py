#!/usr/bin/env python3

import rospy
from std_msgs.msg import *
from duckietown_msgs.msg import *
from PID_class import PID

class Dist_PID(PID):
    def __init__(self,p,i,d,time,vel,om):
        PID.__init__(self,p,i,d,time)
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.vel = vel
        self.om = om
        rospy.set_param("p_d",self.kp)
        rospy.set_param("i_d",self.ki)
        rospy.set_param("d_d",self.kd)

    def callback(self,data):
        tmp_p = rospy.get_param("p_d")
        tmp_i = rospy.get_param("i_d")
        tmp_d = rospy.get_param("d_d")
        if tmp_p!=self.kp or tmp_i!=self.ki or tmp_d!=self.kd or self.integral>1000000000:
            self.integral = 0
            self.changePID(self, tmp_p, tmp_i, tmp_d)

        error = data.d
        timing = rospy.get_rostime()
        temp_time = timing.secs+(timing.nsecs/1000000000)
        rospy.loginfo(("time: {}, d: {}, phi: {}, ".format(temp_time, data.d, data.phi)))

        inp = self.calculateSignal(error,temp_time)
        #if inp!=0:
        #    self.vel = 0
        #    self.move(self, 0, (self.om*10))
        #else:
        #    self.move(self, self.vel, 0)
        if inp>0:
            om = -self.om
        elif inp<0:
            om = self.om
        else:
            om=0
        vel = self.vel

        self.move(vel,om)


    def move(self,vel,om):
        move_msg = Twist2DStamped()
        move_msg.v = vel
        move_msg.omega = om

        self.pub.publish(move_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('dist_pid_controller_node')
        timing = rospy.get_rostime()
        Dist_PID(0.185, 0, 0, (timing.secs+(timing.nsecs)), 0.2, 4)
        #hw6 values: p=0.185, i=0.00009, d=1.9

        rospy.spin()
        #rate=rospy.Rate(10)

        #for count in range(0,100):#10 second timer
        #    ob.move(0.7,1.5)
        #    rate.sleep()
        #ob.move(0,0)
    except rospy.ROSInterruptException:
        pass



