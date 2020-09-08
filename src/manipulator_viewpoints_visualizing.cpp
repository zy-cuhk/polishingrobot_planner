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
#include "../include/aubo_kinematics.h"
#include <jsoncpp/json/json.h> 



#include <ros/ros.h>  
#include "ros/console.h"
#include <sensor_msgs/PointCloud2.h>  

#include <pcl/point_cloud.h>  
#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h>

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

using namespace std;


int main(int argc,char**argv)
{
    // initialize ros node and setup ros topics
	ros::init (argc, argv, "viewpoint_planning_results_visualization");  
	ros::NodeHandle nh;  
    int hz=2;
	ros::Rate loop_rate(hz);

	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/pointcloud/output", 10);  
	sensor_msgs::PointCloud2 output;  
	output.header.stamp=ros::Time::now();
	output.header.frame_id ="map";

    ros::Publisher octomapPublisher = nh.advertise<octomap_msgs::Octomap>("octomap", 1, false);
    octomap_msgs::Octomap octomapMsg;
    octomapMsg.header.frame_id = "map";

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber sub = nh.subscribe("/joint_states", 10, jointstatesCallback);

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

    // phase 1: preset for next best view alogrithm 
    // phase 1-step 1: obtiain points cloud of covered wall workspace is shown as follows:
    pcl::PointCloud<pcl::PointXYZ> cloud;
    float cell_width= 0.8246;
    float wall2mobileplatformbase_distance=1.0;
    float wall_height =2.70;
    cloud.width=24;
    cloud.height=76;
    cloud.is_dense=false;
    cloud.points.resize(cloud.width*cloud.height);
    int ii, iii;
    for(size_t i=0;i<cloud.points.size();++i)
    {
        ii=int(i/cloud.height);
        iii=int(i%cloud.height);

        cloud.points[i].x=wall2mobileplatformbase_distance;
        cloud.points[i].y=(-cell_width/2)+(ii*0.03489);
        cloud.points[i].z=(0.0)+(iii*0.03574);
    }

    // phase 1-step2: create octree from point cloud of covered wall workspace
    float cloudCentroid[3]={0,0,0};
    octomap::OcTree cloudAndUnknown2(0.01);

    // phase 1-step3: create octree of camera FOV 
    octomap::point3d Point3dwall(0.0,0.0,1.20);    
    octomap::Pointcloud pointwall;     
    for(int ii=1;ii<283;ii++){
        for(int iii=1;iii<173;iii++){
            Point3dwall.x()= (-0.4920)+(ii*0.003489);
            Point3dwall.y()= (-0.3080)+(iii*0.003574);
            pointwall.push_back(Point3dwall);
        }
    }

    float manipulatorbase_position[6]={0.18, 0.0, 1.196, 0.0, 0.0, 0.0};

    Json::Value onecellviewpoints_candidatejointsolutions_dict, onecellviewpoints_position_dict;
    Json::Reader reader;
    std::ifstream ifs("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/src/onecellviewpoints_candidatejointsolutions_dict.json");
    reader.parse(ifs, onecellviewpoints_candidatejointsolutions_dict);

    std::ifstream ifs1("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/src/onecellviewpoints_position_dict.json");
    reader.parse(ifs1, onecellviewpoints_position_dict);


    // float aubo_q1[6];
    // for (int i=0; i<onecellviewpoints_candidatejointsolutions_dict.size(); i++){
    //     std::string str1=to_string(i)+"th_selected_viewpoint";
    //     for (int j=0; j<onecellviewpoints_candidatejointsolutions_dict[str1].size(); j++){
    //         std::string str2=to_string(j)+"th_candidate_joint_solution";
    //         for (int k=0; k<onecellviewpoints_candidatejointsolutions_dict[str1][str2].size(); k++){
    //             aubo_q1[k] = onecellviewpoints_candidatejointsolutions_dict[str1][str2][k].asFloat()*180/M_PI;
    //         }
    //     }
    // }

    std::ofstream ofs("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/src/onecellviewpoints_candidatejointsolutions_dict.json");
    ofs << onecellviewpoints_candidatejointsolutions_dict;
    ofs.close();

    //--------------------------------------------------------------------------------------------------------------------------------------
    // phase 2: visualize robot motion and camera coverage viewing 
    while (ros::ok())
    {
        // publish the cloud point 
        pcl_pub.publish(output);

        // pubilish the original octomsg
        for (size_t i = 0; i < cloud.points.size (); ++i){ 
            octomap::OcTreeNode * cloudNode2=cloudAndUnknown2.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
            cloudNode2->setValue(1);
        }
        octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg);
        octomapPublisher.publish(octomapMsg);
        loop_rate.sleep();  
        ros::spinOnce(); 

        // publish joint states and changed octomap states 
        std::string str1, str2;
        for (int i=0; i<onecellviewpoints_candidatejointsolutions_dict.size(); i++){
            str1=to_string(i)+"th_selected_viewpoint";
            for (int j=0; j<onecellviewpoints_candidatejointsolutions_dict[str1].size(); j++){
                // publish the joint states 
                joint_state.header.stamp = ros::Time::now();
                joint_state.position[0] = 0.0;
                joint_state.position[1] = 0.0;
                joint_state.position[2] = 0.0;
                joint_state.position[3] = 0.0;

                str2=to_string(j)+"th_candidate_joint_solution";
                for (int k=0; k<onecellviewpoints_candidatejointsolutions_dict[str1][str2].size(); k++){
                    joint_state.position[4+k] = onecellviewpoints_candidatejointsolutions_dict[str1][str2][k].asFloat();
                }
                joint_pub.publish(joint_state);

                loop_rate.sleep();  
                ros::spinOnce();  
            }
            // publish the coverage state of octree cloudAndUnknown2
            std::string str3=to_string(i)+"th_selected_viewpointposition";
            float aubo_viewposition[6];
            for (int k=0;k<6;k++){
                aubo_viewposition[k]=onecellviewpoints_position_dict[str3][k].asFloat();
            }

            octomap::Pointcloud variablePointwall;     
            octomap::point3d iterator; 
            iterator.x()=aubo_viewposition[0]+manipulatorbase_position[0];        
            iterator.y()=aubo_viewposition[1]+manipulatorbase_position[1];    
            iterator.z()=aubo_viewposition[2]+manipulatorbase_position[2];    
            
            float roll, pitch, yaw;
            roll=aubo_viewposition[3]+manipulatorbase_position[3];  
            pitch=aubo_viewposition[4]+manipulatorbase_position[4];  
            yaw=aubo_viewposition[5]+manipulatorbase_position[5];  

            octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
            octomath::Quaternion Rotation2(roll,pitch,yaw);	
            octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
            variablePointwall=pointwall;		
            variablePointwall.transform(RotandTrans2);

            octomap::KeyRay rayBeam;
            int unknownVoxelsInRay=0;
            int known_points_projection=0;
            for (int ii=0; ii<variablePointwall.size();ii++){
                bool Continue=true;		
                cloudAndUnknown2.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
                for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                    octomap::OcTreeNode * node=cloudAndUnknown2.search(*it);	
                    if(node!=NULL){
                        cloudAndUnknown2.updateNode(*it, false);
                        Continue=false;
                    }
                }
            }
            cloudAndUnknown2.updateInnerOccupancy();
            octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg);
            octomapPublisher.publish(octomapMsg);

            loop_rate.sleep();  
            ros::spinOnce(); 
        }
    }
    return 0;
}