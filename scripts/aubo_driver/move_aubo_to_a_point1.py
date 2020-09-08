#! /usr/bin/env python
# coding=utf-8
import math
import json
from aubo_robotcontrol import *
from aubo_kienamtics import *
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)
def rad_to_degree(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*180/math.pi)
    return tuple(dd)
def main(test_count):
    # 初始化logger
    logger_init()

    # 启动测试
    logger.info("{0} test beginning...".format(Auboi5Robot.get_local_time()))

    # 系统初始化
    Auboi5Robot.initialize()

    # 创建机械臂控制类
    robot = Auboi5Robot()

    # 创建上下文
    handle = robot.create_context()

    # 打印上下文
    logger.info("robot.rshd={0}".format(handle))
    try:

        # 链接服务器
        ip = '192.168.1.115'
        port = 8899
        result = robot.connect(ip, port)

        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            # # 重新上电
            # robot.robot_shutdown()
            #
            # # 上电
            robot.robot_startup()
            #
            # # 设置碰撞等级
            # robot.set_collision_class(7)

            # 设置工具端电源为１２ｖ
            # robot.set_tool_power_type(RobotToolPowerType.OUT_12V)

            # 设置工具端ＩＯ_0为输出
            robot.set_tool_io_type(RobotToolIoAddr.TOOL_DIGITAL_IO_0, RobotToolDigitalIoDir.IO_OUT)

            # 获取工具端ＩＯ_0当前状态
            tool_io_status = robot.get_tool_io_status(RobotToolIoName.tool_io_0)
            # logger.info("tool_io_0={0}".format(tool_io_status))

            # 设置工具端ＩＯ_0状态
            robot.set_tool_io_status(RobotToolIoName.tool_io_0, 1)

            # 获取控制柜用户DI
            io_config = robot.get_board_io_config(RobotIOType.User_DI)

            # 输出DI配置
            logger.info(io_config)

            # 获取控制柜用户DO
            io_config = robot.get_board_io_config(RobotIOType.User_DO)

            # 输出DO配置
            logger.info(io_config)

            # 当前机械臂是否运行在联机模式
            # logger.info("robot online mode is {0}".format(robot.is_online_mode()))

            # 循环测试
            while test_count > 0:
                test_count -= 1

                joint_status = robot.get_joint_status()
                # logger.info("joint_status={0}".format(joint_status))

                # 初始化全局配置文件
                robot.init_profile()

                # 设置关节最大加速度
                # robot.set_joint_maxacc((5.5, 5.5, 5.5, 5.5, 5.5, 5.5))
                #
                # # 设置关节最大加速度
                # robot.set_joint_maxvelc((1.5, .5, 2.5, 2.5, 2.5, 2.5))
                # 设置关节最大加速度
                # robot.set_joint_maxacc((0.05, 0.05, 0.05, 0.05, 0.05, 0.05))

                # # 设置关节最大加速度
                # robot.set_joint_maxvelc((0.05, 0.05, 0.05, 0.05, 0.05, 0.05))
                acc_data=0.3
                robot.set_joint_maxacc((acc_data,acc_data,acc_data,acc_data,acc_data,acc_data))

                # 设置关节最大加速度
                velc_data=0.3
                robot.set_joint_maxvelc((velc_data,velc_data,velc_data,velc_data,velc_data,velc_data))
                # 设置机械臂末端最大线加速度(m/s)
                robot.set_end_max_line_acc(0.3)
                logger.info("-------go-----to-----start-------step--01")
                # 获取机械臂末端最大线加速度(m/s)
                # robot.set_end_max_line_velc(0.2)
                robot.set_end_max_line_velc(0.3)

                with open("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/scripts/moveit_planning_trajectory.json",'r') as f:
                    trajectory=json.load(f)
                # for i in range(len(trajectory)):
                i=2
                str1="trajectory_num_"+str(i)
                waypoint_list=trajectory[str1]
                print("the waypoints number is:",len(waypoint_list))
                robot.move_joint((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                robot.set_blend_radius(blend_radius=0.05)
                robot.remove_all_waypoint()
                for i in range(len(waypoint_list)):
                    aubo_joints=waypoint_list["waypoints_num_"+str(i)]
                    print("aubo joints are:",aubo_joints)
                    joint_radian = (aubo_joints[0], aubo_joints[1], aubo_joints[2], aubo_joints[3], aubo_joints[4], aubo_joints[5])
                    robot.add_waypoint(joint_radian)
                    # robot.move_joint(joint_radian)
                flag=robot.move_track(RobotMoveTrackType.CARTESIAN_MOVEP)
                if flag:
                    rospy.logerr("movet command work successfully")
                else:
                    rospy.logerr("movet command doesn't work")





            # 断开服务器链接
            robot.disconnect()

    except RobotError, e:
        logger.error("{0} robot Event:{1}".format(robot.get_local_time(), e))

    finally:
        # 断开服务器链接
        if robot.connected:
            # 关闭机械臂
            # robot.robot_shutdown()
            # 断开机械臂链接
            robot.disconnect()
        # 释放库资源
        Auboi5Robot.uninitialize()
        logger.info("{0} test completed.".format(Auboi5Robot.get_local_time()))

if __name__=="__main__":
    main(1)
    # print deg_to_rad((-3.3364,12.406,-81.09,-91.207,-86.08,0.164))8