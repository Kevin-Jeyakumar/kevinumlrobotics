#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import *
from PID_class import PID

class PID_Controller_v2():
    def __init__(self,time):
        #hw6 values: p=0.185, i=0.00009, d=1.9
        self.d_obj = PID(1,0,0,time)
        self.phi_obj = PID(1,0,0,time)
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback)
        self.pub = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.pub1 = rospy.Publisher("/my_dist_error", Float32, queue_size=10)
        self.pub2 = rospy.Publisher("/my_phi_error", Float32, queue_size=10)
        self.pub3 = rospy.Publisher("/my_desired", Float32, queue_size=10)
        self.vel = 0.1
        self.om = 0.6
       
        rospy.set_param("/p_d",self.d_obj.kp)
        rospy.set_param("/i_d",self.d_obj.ki)
        rospy.set_param("/d_d",self.d_obj.kd)

        rospy.set_param("/p_phi",self.phi_obj.kp)
        rospy.set_param("/i_phi",self.phi_obj.ki)
        rospy.set_param("/d_phi",self.phi_obj.kd)
       
    def callback(self,data):
        tmp_d_p = rospy.get_param("/p_d")
        tmp_d_i = rospy.get_param("/i_d")
        tmp_d_d = rospy.get_param("/d_d")
        tmp_phi_p = rospy.get_param("/p_phi")
        tmp_phi_i = rospy.get_param("/i_phi")
        tmp_phi_d = rospy.get_param("/d_phi")
        self.d_obj.changePID(tmp_d_p, tmp_d_i, tmp_d_d)
        self.phi_obj.changePID(tmp_phi_p, tmp_phi_i, tmp_phi_d)
        if self.d_obj.integral>1000000000:
            self.d_obj.integral = 0
        if self.phi_obj.integral>1000000000:
            self.phi_obj.integral = 0

        d_error = 0 - data.d
        phi_error = 0 - data.phi
        timing = rospy.get_rostime()
        temp_time = timing.secs+(timing.nsecs/1000000000)

        temp_time_diff = temp_time - self.d_obj.past_time_stamp
        d_inp = self.d_obj.calculateSignal(d_error,temp_time)
        phi_inp = self.phi_obj.calculateSignal(phi_error,temp_time)
        rospy.loginfo(("time: {:2.3f}, d: {:2.3f}, phi: {:2.3f}, d_input: {:2.3f}, phi_input: {:2.3f}".format(temp_time_diff, data.d, data.phi, d_inp, phi_inp)))
        self.pub1.publish(d_error)
        self.pub2.publish(phi_error)
        self.pub3.publish(0)

        vel = self.vel
        om = d_inp + phi_inp
        if om<0.1 and om>-0.1:
            om = 0
        #elif om>2:
        #    om = 2
        #elif om<-2:
        #    om = -2 
        if om!=0:
            vel=0

        self.move(vel,om)


    def move(self,vel,om):
        move_msg = Twist2DStamped()
        move_msg.v = vel
        move_msg.omega = om

        self.pub.publish(move_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('pid_controller_node_v2')
        timing = rospy.get_rostime()
        PID_Controller_v2((timing.secs+(timing.nsecs/1000000000)))

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


