#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
import time
import numpy
import os
import yaml
import numpy as np
import numpy.matlib
import json

from std_msgs.msg import String,Float64,Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory
from aubo_kienamatics import *

class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.planning_sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.trajectory_planning_callback, queue_size=1)
        self.planning_list=[]

    def trajectory_planning_callback(self,data):
        self.planning_list=[]
        for i in range(len(data.trajectory[0].joint_trajectory.points)):
            temp_list=[]
            count_zero=0
            for j in range(len(data.trajectory[0].joint_trajectory.points[i].positions)):
                if abs(data.trajectory[0].joint_trajectory.points[i].positions[j])<0.0000000001:
                    count_zero+=1
            if count_zero>=3:
                for j in range(len(data.trajectory[0].joint_trajectory.points[i].positions)):
                    if abs(data.trajectory[0].joint_trajectory.points[i].positions[j])<0.0001:
                        temp_list.append(0.0)
                    else:
                        temp_list.append(data.trajectory[0].joint_trajectory.points[i].positions[j])
            self.planning_list.append((0,0,0,0,0,0)+tuple(temp_list))
    def print_planning_result():
        if len(self.planning_list)~=0:
            
        

def main():
    ratet=1
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
            
        rate.sleep()
if __name__ == '__main__':
    main()