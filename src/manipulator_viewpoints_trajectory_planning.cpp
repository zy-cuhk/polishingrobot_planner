#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <assert.h>
#include <jsoncpp/json/json.h> 

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_aubo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "aubo10";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  Json::Value onecellviewpoints_selectedjointsolution_dict;
  Json::Reader reader;
  std::ifstream ifs("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/src/onecellviewpoints_selectedjointsolutions_dict.json");
  reader.parse(ifs, onecellviewpoints_selectedjointsolution_dict);

  

  // while (ros::ok()){
    for (int i=0; i<onecellviewpoints_selectedjointsolution_dict.size(); i++){
        std::string str1=to_string(i)+"th_selected_viewpoint";
        for (int j=0; j<onecellviewpoints_selectedjointsolution_dict[str1].size(); j++){
            std::string str2=to_string(j)+"th_candidate_joint_solution";
            for (int k=0; k<onecellviewpoints_selectedjointsolution_dict[str1][str2].size(); k++){
              joint_group_positions[k] = onecellviewpoints_selectedjointsolution_dict[str1][str2][k].asFloat();
            }
            cout<<"the joint group position is: "<<joint_group_positions[0]<<" "<<joint_group_positions[1]<<" "<<joint_group_positions[2]<<" "<<joint_group_positions[3]<<" "<<joint_group_positions[4]<<" "<<joint_group_positions[5]<<endl;
            move_group.setJointValueTarget(joint_group_positions);
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("the cartesian position number is: %s", str1.c_str());
            ROS_INFO_NAMED("the joint solution number is: %s", str2.c_str()); 
            ROS_INFO_NAMED("tutorial", "Visualizing plan (joint space goal) %s", success ? "SUCCESS" : "FAILED");
            
            // collisiion state publish
            if(ros::param::has("/judge_planningsuccess_flag")){   
              if (success==true){
                ros::param::set("/judge_planningsuccess_flag",1);
              }
              else{
                ros::param::set("/judge_planningsuccess_flag",0);
              }
            }
            cout<<"-------------------------------------------------------------"<<endl;
            sleep(2);
            if(ros::param::has("/judge_planningsuccess_flag")){   
              if (success==true){
                ros::param::set("/judge_planningsuccess_flag",0);
              }
            }
        }
    }
  // }


  ros::shutdown();
  return 0;
}
