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

class AuboCollisionCheck():
    def __init__(self):
        self.pub_state_ = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.planning_sub = rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.trajectory_planning_callback1, queue_size=1)
        self.planning_list=[]

    def Init_node(self):
        rospy.init_node("aubocollisioncheck")

    def trajectory_planning_callback1(self,data):
        self.planning_list=[]
        for i in range(len(data.trajectory[0].joint_trajectory.points)):
            for j in range(len(data.trajectory[0].joint_trajectory.points[i].positions)):
                self.planning_list.append(data.trajectory[0].joint_trajectory.points[i].positions[j])
    def print_planning_result(self):
        planning_jointlists=np.zeros((int(len(self.planning_list)/6),6))
        for i in range(len(self.planning_list)):
            m=int(i/6)
            n=i%6
            planning_jointlists[m][n]=self.planning_list[i]
        return planning_jointlists
            
        

def main():
    ratet=1
    Aub=AuboCollisionCheck()
    Aub.Init_node()

    waypoint={}
    trajectory={}
    trajectory_num=0
    rate = rospy.Rate(ratet)
    while not rospy.is_shutdown():
        judge_planningsuccess_flag=rospy.get_param("/judge_planningsuccess_flag")
        if judge_planningsuccess_flag==1:
            print("---------------------------------------------------------------------------")
            planning_jointlists=Aub.print_planning_result()
            if len(planning_jointlists)!=0:
                for i in range(len(planning_jointlists)):
                    print("the joint list is:",planning_jointlists[i][0],planning_jointlists[i][1],planning_jointlists[i][2],planning_jointlists[i][3],planning_jointlists[i][4],planning_jointlists[i][5])
                    waypoint.update({("waypoints_num_"+str(i)):planning_jointlists[i].tolist()})
                trajectory.update({("trajectory_num_"+str(trajectory_num)):waypoint})
                trajectory_num=trajectory_num+1
                waypoint={}
            print("----------------------------------------------------------------------------")
            with open("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/scripts/moveit_planning_trajectory.json",'w') as f:
                json.dump(trajectory, f, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False)
        rate.sleep()



if __name__ == '__main__':
    main()