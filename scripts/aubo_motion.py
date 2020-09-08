#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import time
import os
import math
import re
import numpy as np
from std_msgs.msg import String,Float64,Bool
from sensor_msgs.msg import JointState
class Renovation_operation():
    def __init__(self):
        self.current_joints=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.default_start_joints=rospy.get_param('/renov_up_level/aubo_default_point')
        self.default_end_joints=rospy.get_param("/renov_up_level/aubo_default_point")
        # self.default_start_joints2=[0.0,0.7156,2.7524,0.47,-1.487,1.57]
        # self.default_end_joints2=[0.0,0.7156,2.7524,0.47,-1.487,0] # rospy.get_param("/renov_up_level/aubo_end_point")
        self.tolerance_tracking_error=0.1
        self.aubo_move_track_pub=rospy.Publisher('/aubo_ros_script/movet', String, queue_size=1)
        self.aubo_move_joint_pub = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=1)
        self.aubo_move_line_pub = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=1)
        self.aubo_joints_sub=rospy.Subscriber('/renov_up_level/aubo_joints',JointState,self.obtain_aubo_joints,queue_size=10)
    def group_joints_to_string(self,q_list):
        group_joints=""
        for i in range(len(q_list)):
            group_joints+=str(tuple(q_list[i]))
        return group_joints
    def obtain_aubo_joints(self,msg):
        self.current_joints=msg.position[:]

    def manipulator_motion_simulation(self,aubo_q_list,rate):
        self.motion_state_current2last()
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])
        # rospy.loginfo("aubo joints are: %s",aubo_joints)
        pubstring="movet"+self.group_joints_to_string(aubo_joints)+self.default_end_joints+self.default_start_joints
        # rospy.loginfo("the published string is: %s",pubstring)
        count=1
        while not rospy.is_shutdown():
            last_motion_phase_over_flag=rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
            # rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)
            if last_motion_phase_over_flag==1:
                rospy.logerr("step 4: manipulator_renovation_motion is in process")
                # self.aubo_move_track_pub.publish(pubstring)
                os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 0") 

                renovation_tool_tracking_error_01=0.0
                renovation_tool_tracking_error_02=0.0
                manipulator_operation_tracking_error=0.0
                tolerance_tracking_error=0.01
                if abs(renovation_tool_tracking_error_01)<=tolerance_tracking_error:
                    rospy.logerr("the motion of electric switch is open")
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
                else:
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 0')
                if abs(renovation_tool_tracking_error_02)<=tolerance_tracking_error:
                    rospy.logerr("the motion of electric switch is closed")
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
                else:
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 0')

                if abs(manipulator_operation_tracking_error)<=tolerance_tracking_error:
                    rospy.logerr("step 4: manipulator_renovation_motion is closed")
                    os.system('rosparam set /renov_up_level/current_motion_phase_over_flag 1')
                    break
            rate.sleep()

    def Tuple_string_to_tuple(self,tuplestring):
        tupletemp = re.findall(r'\-?\d+\.?\d*', tuplestring)
        resdata=[]
        for i in tupletemp:
            resdata.append(float(i))
        return tuple(resdata)

    def motion_state_current2last(self):
        current_motion_phase_over_flag=rospy.get_param("/renov_up_level/current_motion_phase_over_flag")
        if current_motion_phase_over_flag==1:
            os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 1")

    def manipulator_tracking_error_computation(self,refence_joints):
        current_aubo_joints1=np.array(self.current_joints)
        manipulator_operation_tracking_errorlist=refence_joints-current_aubo_joints1 
        manipulator_operation_tracking_error=math.sqrt(np.sum((manipulator_operation_tracking_errorlist)**2))
        return manipulator_operation_tracking_error

    def manipulator_motion(self,pubstring,rate,count):
        self.motion_state_current2last()
        tuplefloatdata=self.Tuple_string_to_tuple(pubstring)
        start_joints=tuplefloatdata[0:6]
        end_joints=tuplefloatdata[len(tuplefloatdata)-6:len(tuplefloatdata)]

        while not rospy.is_shutdown():
            last_motion_phase_over_flag=rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)
            
            if last_motion_phase_over_flag==1:
                if "movej" in pubstring:
                    self.aubo_move_joint_pub.publish(pubstring)
                elif "movel" in pubstring:
                    self.aubo_move_line_pub.publish(pubstring)
                elif "movet" in pubstring:
                    self.aubo_move_track_pub.publish(pubstring)

                rospy.logerr("manipulator motion phase %s is in process"%str(count))
                time.sleep(0.5)
                manipulator_operation_tracking_error= self.manipulator_tracking_error_computation(start_joints)
                rospy.logerr("manipulator motion phase %s starting error is: %s"%(str(count),str(manipulator_operation_tracking_error))) 

                "manipulator motion triggering condition"
                if manipulator_operation_tracking_error>=self.tolerance_tracking_error:
                    os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 0")
                    os.system("rosparam set /renov_up_level/current_motion_phase_start_flag 1")
                else:
                    pass
            
            current_motion_start_flag=rospy.get_param("/renov_up_level/current_motion_phase_start_flag")
            if current_motion_start_flag==1:
                manipulator_operation_tracking_error= self.manipulator_tracking_error_computation(end_joints)
                rospy.logerr("manipulator motion phase %s stoping error is: %s"%(str(count),str(manipulator_operation_tracking_error)))
                
                "nonrenovation motion termination condition"
                if abs(manipulator_operation_tracking_error)<=self.tolerance_tracking_error:
                    rospy.logerr("manipulator motion phase %s is terminated"%str(count))
                    os.system("rosparam set /renov_up_level/current_motion_phase_start_flag 0")
                    os.system("rosparam set /renov_up_level/current_motion_phase_over_flag 1")
                    break
            rate.sleep()

    def painting_gun_open_control(self):
        self.motion_state_current2last()
        while not rospy.is_shutdown():
            last_motion_phase_over_flag=rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)

            if last_motion_phase_over_flag==1:
                rospy.logerr("the motion of electric switch is open")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
                rospy.sleep(0.1)
                electric_switch_painting_open_state=rospy.get_param("/renov_up_level/write_electric_switch_painting_open")
                "painting gun triggering condition"
                if electric_switch_painting_open_state==1:
                    os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 0")
                    os.system("rosparam set /renov_up_level/current_motion_phase_start_flag 1")
                else:
                    pass

            current_motion_start_flag=rospy.get_param("/renov_up_level/current_motion_phase_start_flag")
            if current_motion_start_flag==1:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 0')
                os.system("rosparam set /renov_up_level/current_motion_phase_start_flag 0")
                os.system("rosparam set /renov_up_level/current_motion_phase_over_flag 1")
                break
            rate.sleep()            

    def painting_gun_close_control(self):
        self.motion_state_current2last()
        while not rospy.is_shutdown():
            last_motion_phase_over_flag=rospy.get_param("/renov_up_level/last_motion_phase_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('last_motion_phase_over_flag'), last_motion_phase_over_flag)

            if last_motion_phase_over_flag==1:
                rospy.logerr("the motion of electric switch is close")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
                rospy.sleep(0.1)
                electric_switch_painting_close_state=rospy.get_param("/renov_up_level/write_electric_switch_painting_close")
                "painting gun triggering condition"
                if electric_switch_painting_close_state==1:
                    os.system("rosparam set /renov_up_level/last_motion_phase_over_flag 0")
                    os.system("rosparam set /renov_up_level/current_motion_phase_start_flag 1")
                else:
                    pass

            current_motion_start_flag=rospy.get_param("/renov_up_level/current_motion_phase_start_flag")
            if current_motion_start_flag==1:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 0')
                os.system("rosparam set /renov_up_level/current_motion_phase_start_flag 0")
                os.system("rosparam set /renov_up_level/current_motion_phase_over_flag 1")
                break
            rate.sleep()      

    def aubo_motion(self,aubo_q_list,rate):
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])

        pubstring1="movej"+self.default_start_joints+self.group_joints_to_string(aubo_joints[0:1])
        pubstring2="movet"+self.group_joints_to_string(aubo_joints[1:len(aubo_joints)])
        pubstring3="movej"+self.group_joints_to_string(aubo_joints[len(aubo_joints)-1:len(aubo_joints)])+self.default_end_joints

        print("pubstring1=%s"%pubstring1)
        print("pubstring2=%s"%pubstring2)
        print("pubstring3=%s"%pubstring3)

        count=1
        self.manipulator_motion(pubstring1,rate,count)
       # self.painting_gun_open_control()
        count=count+1
        self.manipulator_motion(pubstring2,rate,count)
        # self.painting_gun_close_control()
        count=count+1
        self.manipulator_motion(pubstring3,rate,count)

    def aubo_motion1(self, aubo_q_list, rate):
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])

        "motion of manipulator to start point"
        count=1
        pubstring1="movej"+self.default_start_joints+self.group_joints_to_string(aubo_joints[0:1])
        print("the beginning motion pubstring1=%s"%pubstring1)
        self.manipulator_motion(pubstring1,rate,count)
        count=count+1

        "motion of manipulator to waypoints lines"
        for i in range(len(aubo_joints)-1):
            if i%2==0:
                self.painting_gun_open_control()
                pubstring="movel"+self.group_joints_to_string(aubo_joints[i:i+1])+self.group_joints_to_string(aubo_joints[i+1:i+2])
                print("the painting process is pubstring%s is %s"%(str(count),pubstring))
                self.manipulator_motion(pubstring,rate,count)
                self.painting_gun_close_control()
                count=count+1
            else:
                pubstring="movel"+self.group_joints_to_string(aubo_joints[i:i+1])+self.group_joints_to_string(aubo_joints[i+1:i+2])
                print("the non-painting process is pubstring%s is %s"%(str(count),pubstring))
                self.manipulator_motion(pubstring,rate,count)
                count=count+1

        "motion of manipulator to end points"
        pubstring3="movej"+self.group_joints_to_string(aubo_joints[len(aubo_joints)-1:len(aubo_joints)])+self.default_end_joints
        print("the ending motion pubstring3=%s"%pubstring3)
        self.manipulator_motion(pubstring3,rate,count)




def main():
    nodename="renovation_operation"
    rospy.init_node(nodename)
    ratet=30
    rate=rospy.Rate(ratet)

    aubo_q_list={"aubo_data_num_0": [-0.28525098, -0.53203763, 1.36669062, -1.24286441, -1.85604731, 1.57079633], "aubo_data_num_1": [0.71039368, -0.53203763, 1.36669062, -1.24286441, -0.86040264, 1.5707963], "aubo_data_num_2": [0.71039368, -0.63763321, 1.4856621, -1.01829734, -0.86040264, 1.57079633], "aubo_data_num_3": [-0.28525098, -0.63763321, 1.4856621, -1.01829734, -1.85604731, 1.57079633], "aubo_data_num_4": [-0.28525098, -0.78704025, 1.5382336, -0.8163188, -1.85604731, 1.57079633], "aubo_data_num_5": [0.71039368, -0.78704025, 1.5382336, -0.8163188, -0.86040264, 1.57079633], "aubo_data_num_6": [0.71039368, -0.96986677, 1.52551268, -0.64621321, -0.86040264, 1.57079633]}
    # aubo_joints=[]
    # for i in range(len(aubo_q_list)):
    #     aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])
    # print(aubo_joints[0:1])
    # print(aubo_joints[1:len(aubo_joints)-1])
    # print(aubo_joints[len(aubo_joints)-1:len(aubo_joints)])
    # group_joints=""
    # for i in range(len(aubo_joints)):
    #     group_joints+=str(tuple(aubo_joints[i]))
    # print(group_joints)
    aubo5=Renovation_operation()
    aubo5.aubo_motion(aubo_q_list,rate)

    # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
    
if __name__=="__main__":
    main()

