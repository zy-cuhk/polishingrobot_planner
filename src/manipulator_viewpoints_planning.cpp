// the function of this cpp is to generate a viewpoint for camera and then obtain the covered octomap and visualize the octoma
// step one: generate a viewpoint 
// step two: obtain the covered octomap 
// step three: visualize the changed octoamp in RVIZ

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <assert.h>

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

using namespace std;

int main(int argc,char**argv)
{
    // ros node initialization
	ros::init (argc, argv, "viewpoint_planning_using_octomap");  
	ros::NodeHandle nh;  

    // build up the simplified wall model in pcd file format 
    pcl::PointCloud<pcl::PointXYZ> new_cloud;
    
    float cell_width= 0.8246;
    float wall2mobileplatformbase_distance=1.0;
    float wall_height =2.70;

    new_cloud.width=32;
    new_cloud.height=76;
    new_cloud.is_dense=false;
    new_cloud.points.resize(new_cloud.width*new_cloud.height);

    int ii, iii;
    for(size_t i=0;i<new_cloud.points.size();++i)
    {
        ii=int(i/new_cloud.height);
        iii=int(i%new_cloud.height);

        new_cloud.points[i].x=wall2mobileplatformbase_distance;
        new_cloud.points[i].y=(-cell_width/2)+(ii*0.03489);
        new_cloud.points[i].z=(0.0)+(iii*0.03574);
    }
    // write pcd 
    pcl::io::savePCDFileASCII("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/data/test_pcd.pcd",new_cloud);


    // set up pcl rostopic and transform pcd to point cloud 
	std::string topic,path,frame_id;
    int hz=5;
    nh.param<std::string>("path", path, "/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/data/test_pcd.pcd");
	nh.param<std::string>("frame_id", frame_id, "map");
	nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("hz", hz, 5);
	ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> (topic, 10);  
	sensor_msgs::PointCloud2 output;  
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // set up point cloud rostopic 
    pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud); 
	pcl::toROSMsg(*cloud,output);

	output.header.stamp=ros::Time::now();
	output.header.frame_id  =frame_id;
	ros::Rate loop_rate(hz);

    // set up octomap rostopic 
    bool isLatch = false;
    auto octomapPublisher = nh.advertise<octomap_msgs::Octomap>("octomap", 1, isLatch);
    octomap_msgs::Octomap octomapMsg;


    // create octree from point cloud 
    float cloudCentroid[3]={0,0,0};
    octomap::OcTree cloudAndUnknown(0.05);
    for (size_t i = 0; i < cloud->points.size (); ++i){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x+cloudCentroid[0],cloud->points[i].y+cloudCentroid[1],cloud->points[i].z+cloudCentroid[2],true);
        cloudNode->setValue(13);
    }

    //visualize pcd document octree
    octomap::Pointcloud scan;
    octomap::OcTree tree(0.05);
    octomap::point3d sensorOrigin(0,0,0);
    for (size_t i =0; i < cloud->points.size (); ++i){
        scan.push_back(cloud->points[i].x+cloudCentroid[0],cloud->points[i].y+cloudCentroid[1],cloud->points[i].z+cloudCentroid[2]);
    }
    tree.insertPointCloud(scan, sensorOrigin);

    octomapMsg.header.frame_id = "map";
    octomap_msgs::binaryMapToMsg(cloudAndUnknown, octomapMsg);
    while (ros::ok())
    {
        octomapPublisher.publish(octomapMsg);
        pcl_pub.publish(output);  
        loop_rate.sleep();  
        //ros::spinOnce();  
    }

    return 0;

}


