#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
 
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  float pos[3],vel[3];
 // pos=msg.position;
  pos[0]=msg->position[0];
  pos[1]=msg->position[1];
  pos[2]=msg->position[2];
  // vel[0]=msg->velocity[0];
  // vel[1]=msg->velocity[1];
  // vel[2]=msg->velocity[2];
  ROS_INFO("I heard: [%f] [%f] [%f]",pos[0],pos[1],pos[2]); //,vel[0],vel[1],vel[2]);
 
}
 
int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener"); 
  ros::NodeHandle n;
  ros::Rate loop_rate(10);  


  ros::Subscriber sub = n.subscribe("/joint_states", 10, jointstatesCallback);
  ros::spin();
  //loop_rate.sleep();

  return 0;
}