#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from std_msgs.msg import String
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def aubo_ros_test():
    joint_radian1 = (0.3, 0.0, 0.0, 0.0, 0.0, 0.0)
    joint_radian2 = (0.71039368, -0.53203763, 1.36669062, -1.24286441, -0.86040264, 1.5707963)
    joint_radian3 = (0.71039368, -0.63763321, 1.4856621, -1.01829734, -0.86040264, 1.57079633)
    joint_radian4 = (-0.28525098, -0.63763321, 1.4856621, -1.01829734, -1.85604731, 1.57079633)
    
    joint_radian5 = (-0.28525098, -0.78704025, 1.5382336, -0.8163188, -1.85604731, 1.57079633)
    joint_radian6 = (0.71039368, -0.78704025, 1.5382336, -0.8163188, -0.86040264, 1.57079633)
    joint_radian7 = (0.71039368, -0.96986677, 1.52551268, -0.64621321, -0.86040264, 1.57079633)
    joint_radian8 = (-0.28525098, -0.96986677, 1.52551268, -0.64621321, -1.85604731, 1.57079633)
    joint_radian9 = (-0.28525098, -1.17565041, 1.44731776, -0.51862448, -1.85604731, 1.57079633)
    joint_radian10 = (0.71039368, -1.17565041, 1.44731776, -0.51862448, -0.86040264, 1.57079633)
    joint_radian11 = (0.71039368, -1.39770205, 1.3012984, -0.44259221, -0.86040264, 1.57079633)
    joint_radian12 = (-0.28525098, -1.39770205, 1.3012984, -0.44259221, -1.85604731, 1.57079633)

    pub1 = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)
    # pub2 = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
    # pub3 = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=10)
    rospy.init_node('aubo_ros_test', anonymous=True)
    rate = rospy.Rate(20) # 1hz
    while not rospy.is_shutdown():
        movej_points="movej"+str(joint_radian1)
        #movel_points="movel"+str(joint_radian1)+str(joint_radian2)
        # movet_points="movej"+str(joint_radian1)+str(joint_radian2)+str(joint_radian3)+str(joint_radian4)
        # pub1.publish(movej_points)
        # pub2.publish(movel_points)
        pub1.publish(movej_points)
        rate.sleep()

if __name__ == '__main__':
    try:
        aubo_ros_test()
    except rospy.ROSInterruptException:
        pass

