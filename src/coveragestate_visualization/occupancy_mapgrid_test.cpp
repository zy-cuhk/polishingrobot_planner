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
    float cloudCentroid[3]={0,0,0};
    octomap::OcTree cloudAndUnknown2(0.01);
    //--------------------------------------------------------------------------------------------------------------------------------------
    // phase 2: visualize robot motion and camera coverage viewing 
    while (ros::ok())
    {
        // publish the cloud point and original octomsg
        pcl_pub.publish(output);
        for (size_t i = 0; i < cloud.points.size (); ++i){ 
            octomap::OcTreeNode * cloudNode2=cloudAndUnknown2.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
            cloudNode2->setValue(1);
        }
        octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg);
        octomapPublisher.publish(octomapMsg);
        loop_rate.sleep();  
        ros::spinOnce(); 

        // octomap::KeyRay rayBeam;
        // int unknownVoxelsInRay=0;
        // int known_points_projection=0;
        // for (int ii=0; ii<variablePointwall.size();ii++){
        //     bool Continue=true;		
        //     cloudAndUnknown2.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
        //     for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
        //         octomap::OcTreeNode * node=cloudAndUnknown2.search(*it);	
        //         if(node!=NULL){
        //             cloudAndUnknown2.updateNode(*it, false);
        //             Continue=false;
        //         }
        //     }
        // }
        // cloudAndUnknown2.updateInnerOccupancy();
        // octomap_msgs::binaryMapToMsg(cloudAndUnknown2, octomapMsg);
        // octomapPublisher.publish(octomapMsg);

        // loop_rate.sleep();  
        // ros::spinOnce(); 
    }
    return 0;
}