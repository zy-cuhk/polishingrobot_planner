#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty
 
from moveit_msgs.msg import DisplayTrajectory
from tf.transformations import euler_from_quaternion
from collections import deque
import tf
import actionlib
 
import numpy as np
 
import tf2_ros
 
done_waypoints=False

def send_trajectory(waypoints, client=None):
    print(waypoints)


def legacy_get_waypoints(data):
    print(data.trajectory[0].joint_trajectory.points)


if __name__ == '__main__':
    try:
        rospy.init_node('follow_trajectory', anonymous=True)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.Subscriber("/move_group/display_planned_path",DisplayTrajectory,legacy_get_waypoints)
            rate.sleep()


    except rospy.ROSInterruptException:
        print('got exception')