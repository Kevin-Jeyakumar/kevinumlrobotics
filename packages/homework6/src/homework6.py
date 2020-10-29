#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospy

def position_init(x=0,y=0):
  p = np.zeros(2)
  p[0] = x
  p[1] = y
  return p

def transform_matrix(theta_deg,p):
  if theta_deg<0:
    theta_deg += 360
  theta_rad = (theta_deg/360)*2*np.pi # degrees to radians
  R = np.zeros((2,2))
  R[0] = [np.cos(theta_rad), -(np.sin(theta_rad))]
  R[1] = [np.sin(theta_rad), np.cos(theta_rad)]

  H = np.zeros((3,3))
  H[0][0:2] = R[0]
  H[1][0:2] = R[1]
  H[0][2] = p[0]
  H[1][2] = p[1]

  H[2] = [0,0,1]
  return H

def transform_result(H,p):
  p1_appended = np.zeros(3)
  p1_appended[0:2] = p[:]
  p1_appended[2] = 1
  #print("<debug> H:",H)
  #print("<debug> p_appended:",p1_appended)
  p_result = np.matmul(H,np.transpose(p1_appended))

  return p_result[0:2]

if __name__ == "__main__":
    rospy.init_node("homework6_node")
    # 1.a.

    rospy.logwarn("Part 1")
    p00 = position_init()
    p11 = position_init(x=10)
    H01 = transform_matrix(-120,p00) # H01 = [[R01,p00],[0,0,1]]
    pw1 = transform_result(H01,p11) # pw1 = H01*p11
    rospy.logwarn(("pw1:",pw1))

    # 1.b.

    p22 = position_init(x=5)
    H12 = transform_matrix(45,p11) # H12 = [[R12,p11],[0,0,1]]
    pw2 = transform_result(H01,transform_result(H12,p22)) # pw2 = H01*H12*p22
    rospy.logwarn(("pw2:",pw2))

    # 2.a.

    rospy.logwarn("Part 2")
    pwr = position_init(x=3,y=2)
    Hwr = transform_matrix(135,pwr)
    prs = position_init(x=-1,y=0)
    Hrs = transform_matrix(180,prs)
    pws = transform_result(Hwr,prs) # pws = Hwr*prs
    rospy.logwarn(("pwr:",pwr))
    rospy.logwarn(("pws:",pws))

    # 2.b.i.

    psa = position_init(x=4,y=3)
    pra = transform_result(Hrs,psa) # pra = Hrs*psa
    pwa = transform_result(Hwr,pra) # pwa = Hwr*pra
    rospy.logwarn(("pra:",pra))
    rospy.logwarn(("pwa:",pwa))


    # 2.b.ii.

    psb = position_init(x=8,y=2)
    prb = transform_result(Hrs,psb) # prb = Hrs*psb
    pwb = transform_result(Hwr,prb) # pwb = Hwr*prb
    rospy.logwarn(("prb:",prb))
    rospy.logwarn(("pwb:",pwb))

    # 2.c

    parmend = position_init(x=2)
    prarm = position_init()
    Hrarm = transform_matrix(25,prarm) 
    prend = transform_result(Hrarm,parmend) # prend = Hrarm*parmend
    pwend = transform_result(Hwr,prend) # pwend = Hwr*prend
    rospy.logwarn(("prend:",prend))
    rospy.logwarn(("pwend:",pwend))


    try:

        #pyplot only works in colab or other GUI enabled python environment

        plt.figure()
        plt.title("Part 1")
        plt.plot(p00[0],p00[1],'o',label="origin")
        plt.plot(pw1[0],pw1[1],'o',label="bot_position_1")
        plt.plot(pw2[0],pw2[1],'o',label="bot_position_2")
        plt.legend()
        plt.show()
 
        plt.figure()
        plt.title("Part 2")
        plt.plot(p00[0],p00[1],'o',label="origin")
        plt.plot(pwr[0],pwr[1],'o',label="bot_position")
        plt.plot(pws[0],pws[1],'o',label="sensor_position")
        plt.plot(pwa[0],pwa[1],'o',label="point_a")
        plt.plot(pwb[0],pwb[1],'o',label="point_b")
        plt.plot(pwend[0],pwend[1],'o',label="arm_end_position")
        plt.legend()
        plt.show()

    except:
        print("pyplot only works in colab or other GUI enabled python environment")


