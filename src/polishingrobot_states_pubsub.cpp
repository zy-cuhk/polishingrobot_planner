#include<iostream>
#include <string>
#include <ros/ros.h>
#include "../include/aubo_kinematics.h"
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
using namespace std;
float sub_joints[6]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  sub_joints[0]=msg->position[4];
  sub_joints[1]=msg->position[5];
  sub_joints[2]=msg->position[6];
  sub_joints[3]=msg->position[7];
  sub_joints[4]=msg->position[8];
  sub_joints[5]=msg->position[9];
}
 

int main(int argc, char** argv)
 {
    ros::init(argc, argv, "state_1_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, jointstatesCallback);

    ros::Rate loop_rate(10);  


    sensor_msgs::JointState joint_state;
    joint_state.header.frame_id="map";
    joint_state.name.resize(10);
    joint_state.position.resize(10);

    joint_state.name[0]="base_joint1";
    joint_state.name[1]="base_joint2";
    joint_state.name[2]="mobilebase_joint";
    joint_state.name[3]="rodclimbing_joint";
    joint_state.name[4]="shoulder_joint";
    joint_state.name[5]="upperArm_joint";
    joint_state.name[6]="foreArm_joint";
    joint_state.name[7]="wrist1_joint";
    joint_state.name[8]="wrist2_joint";
    joint_state.name[9]="wrist3_joint";

    VectorXd aubo_q(6);
    aubo_q<<0.0, 0.0, 3.14, 0.0, 0.0, 0.0;
    float pub_joints[6]; //={0.0, 0.0, 3.14, 0.0, 0.0, 0.0};
    for (size_t i=0; i<6;i++){
        pub_joints[i]=aubo_q(i);
    }
    bool judge_self_collision_flag;
    int time=0;
    // cout<<"aubo_q is"<<aubo_q(0)<<" "<<aubo_q(1)<<" "<<aubo_q(2)<<" "<<aubo_q(3)<<" "<<aubo_q(4)<<" "<<aubo_q(5)<<endl;
    while (ros::ok()) {
        // if ((aubo_q(0)==sub_joints[0])&&(aubo_q(1)==sub_joints[1])&&(aubo_q(2)==sub_joints[2])&&(aubo_q(3)==sub_joints[3])&&(aubo_q(4)==sub_joints[4])&&(aubo_q(5)==sub_joints[5])){
        if ((pub_joints[0]==sub_joints[0])&&(pub_joints[1]==sub_joints[1])&&(pub_joints[2]==sub_joints[2])&&(pub_joints[3]==sub_joints[3])&&(pub_joints[4]==sub_joints[4])&&(pub_joints[5]==sub_joints[5])){
              ros::param::get("/judge_self_collision_flag", judge_self_collision_flag);
                if (judge_self_collision_flag==0){
                    cout<<"no collision"<<endl;
                }
                else{
                    cout<<"collision"<<endl;
                }
                break;
          }
        else{
            for (int time=0;time<20;time++){
              joint_state.header.stamp = ros::Time::now();
              joint_state.position[0] = 0.0;
              joint_state.position[1] = 0.0;
              joint_state.position[2] = 0.0;
              joint_state.position[3] = 0.0;
              
              joint_state.position[4] = pub_joints[0];
              joint_state.position[5] = pub_joints[1];
              joint_state.position[6] = pub_joints[2];
              joint_state.position[7] = pub_joints[3];
              joint_state.position[8] = pub_joints[4];
              joint_state.position[9] = pub_joints[5];

              joint_pub.publish(joint_state);
              loop_rate.sleep();  
              ros::spinOnce();  
            }
        }     
    }

    return 0;
}













