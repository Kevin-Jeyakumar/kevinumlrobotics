#!/usr/bin/env python3

import sys
import rospy
import actionlib
from example_service.srv import *
import example_action_server.msg

def fibonacci_service_client(x):
    rospy.wait_for_service('calc_fibonacci')
    try:
        calc_fibonacci = rospy.ServiceProxy('calc_fibonacci',Fibonacci)
        respons = calc_fibonacci(x)
        return respons.sequence
    except rospy.ServiceException as e:
        print ("SErvice call failed: ",e)

def fibonacci_action_server_client(x):
    client=actionlib.SimpleActionClient('fibonacci',example_action_server.msg.FibonacciAction)

    client.wait_for_server()

    goal = example_action_server.msg.FibonacciGoal(order=x)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()



if __name__ == "__main__":
#    if len(sys.argv) > 1:
#        x = int(sys.argv[1])

    rospy.init_node('fibonacci_client')

    rospy.logwarn("Started Service Client with arg=3")
    serv_res1=fibonacci_service_client(3)
    print("Service Result for agr=3 : ",serv_res1)
    rospy.logwarn("Service Client Ended arg=3")

    rospy.logwarn("Started Action Server Client with arg=3")
    try:
        act_serv_res1=fibonacci_action_server_client(3)
        print("Action Server Result agr=15 : ",act_serv_res1)
    except rospy.ROSInterruptException:
        print("Action Server Client Interrupted!")
    rospy.logwarn("Action Server Client Ended arg=3")


    rospy.logwarn("Started Service Client with arg=15")
    serv_res2=fibonacci_service_client(15)
    print("Service Result for agr=15 : ",serv_res2)
    rospy.logwarn("Service Client Ended arg=15")

    rospy.logwarn("Started Action Server Client with arg=15")
    try:
        act_serv_res2=fibonacci_action_server_client(15)
        print("Action Server Result for agr=15 : ",act_serv_res2)
    except rospy.ROSInterruptException:
        print("Action Server Client Interrupted")
    rospy.logwarn("Action Server Client Ended arg=15")

    rospy.spin()


