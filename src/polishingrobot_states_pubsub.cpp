// the function is that:
// publish joint states
// subscribe joint states


#include<iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>
using namespace std;
float pos[3];



void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  pos[0]=msg->position[0];
  pos[1]=msg->position[1];
  pos[2]=msg->position[2];
  ROS_INFO("I heard: [%f] [%f] [%f]",pos[0],pos[1],pos[2]); 
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

    while (ros::ok()) {
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = 0.0;
        joint_state.position[1] = 0.0;
        joint_state.position[2] = 0.0;

        joint_state.position[3] = 0.0;
        
        joint_state.position[4] = 0.0;
        joint_state.position[5] = 0.0;
        joint_state.position[6] = 3.15;
        joint_state.position[7] = 0.0;
        joint_state.position[8] = 0.0;
        joint_state.position[9] = 0.0;

        joint_pub.publish(joint_state);
        loop_rate.sleep();  
        ros::spinOnce();  

        cout<<"the reached position is "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<endl;
        //loop_rate.sleep();
    }

    return 0;
}













