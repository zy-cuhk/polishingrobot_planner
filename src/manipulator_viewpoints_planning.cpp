// the function of this cpp is to generate a viewpoint for camera and then obtain the covered octomap and visualize the octoma
// step one: generate a viewpoint 
// step two: obtain the covered octomap 
// step three: visualize the changed octoamp in RVIZ

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
    
    // the parameters from covered wall workspace is shown as follows:
    float cell_width= 0.8246;
    float wall2mobileplatformbase_distance=1.0;
    float wall_height =2.70;

    new_cloud.width=24;
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
    octomap::OcTree cloudAndUnknown(0.01);
    for (size_t i = 0; i < cloud->points.size (); ++i){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud->points[i].x+cloudCentroid[0],cloud->points[i].y+cloudCentroid[1],cloud->points[i].z+cloudCentroid[2],true);
        cloudNode->setValue(1);
    }

    //visualize pcd document octree
    octomap::Pointcloud scan;
    octomap::OcTree tree(0.05);
    octomap::point3d sensorOrigin(0,0,0);
    for (size_t i =0; i < cloud->points.size (); ++i){
        scan.push_back(cloud->points[i].x+cloudCentroid[0],cloud->points[i].y+cloudCentroid[1],cloud->points[i].z+cloudCentroid[2]);
    }
    tree.insertPointCloud(scan, sensorOrigin);


    // create FOV Of camera
    octomap::point3d Point3dwall(0.0,0.0,1.20);    
    octomap::Pointcloud pointwall;     
    // for(int ii=1;ii<321;ii++){
    for(int ii=1;ii<283;ii++){
        for(int iii=1;iii<173;iii++){
            Point3dwall.y()= (-0.4920)+(ii*0.003489);
            Point3dwall.x()= (-0.3080)+(iii*0.003574);
            pointwall.push_back(Point3dwall);
        }
    }

    // sample viewpoint positions 
    int candidate_viewpoints_num = 1;
    int cartesian_freedom = 6;
    float candidate_viewpoint_positions[candidate_viewpoints_num][cartesian_freedom];
    for (int i=0; i<candidate_viewpoints_num; i++){
        candidate_viewpoint_positions[i][0]=0.0;
        candidate_viewpoint_positions[i][1]=0.0;
        candidate_viewpoint_positions[i][2]=0.6;
        candidate_viewpoint_positions[i][3]=0.0;
        candidate_viewpoint_positions[i][4]=M_PI/2; 
        candidate_viewpoint_positions[i][0]=0.0;
    }
    float manipulatorbase_position[6]={0.18, 0.0, 1.196, 0.0, 0.0, 0.0};
    int candidateviewpoints_coveragenode_num[candidate_viewpoints_num];

    // obtain the occupied octomap nodes for candidate viewpoints
    for (int i=0; i<candidate_viewpoints_num; i++){

        // step 1: obtain the pose of camera viewpoint 
        octomap::Pointcloud variablePointwall;     
        octomap::point3d iterator; 
        iterator.x()=candidate_viewpoint_positions[i][0]+manipulatorbase_position[0];        
        iterator.y()=candidate_viewpoint_positions[i][1]+manipulatorbase_position[1];    
        iterator.z()=candidate_viewpoint_positions[i][2]+manipulatorbase_position[2];    
        
        float roll, pitch, yaw;
        roll=candidate_viewpoint_positions[i][3]+manipulatorbase_position[3];  
        pitch=candidate_viewpoint_positions[i][4]+manipulatorbase_position[4];  
        yaw=candidate_viewpoint_positions[i][5]+manipulatorbase_position[5];  

        octomath::Vector3 Translation2(iterator.x(),iterator.y(),iterator.z());		
        octomath::Quaternion Rotation2(roll,pitch,yaw);	
        octomath::Pose6D RotandTrans2(Translation2,Rotation2);	
        variablePointwall=pointwall;		
        variablePointwall.transform(RotandTrans2);

        // step 2: raycast to obtain voxel
        octomap::KeyRay rayBeam;
        int unknownVoxelsInRay=0;
        int known_points_projection=0;
        for (int ii=0; ii<variablePointwall.size();ii++){
            bool Continue=true;		
            cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
            for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                octomap::OcTreeNode * node=cloudAndUnknown.search(*it);	
                if(node!=NULL){
                    cloudAndUnknown.updateNode(*it, false);
                   //cloudAndUnknown.setNodeValue(*it, 1, false);
                    Continue=false;
                }
            }
        }
		cloudAndUnknown.updateInnerOccupancy();

        // test 3: obtian occupied voxel number 
        octomap::point3d iterator1; 
        int uncoverage_points_num=0;
        
        for (size_t i = 0; i < cloud->points.size (); ++i){ 
            iterator1.x()=cloud->points[i].x+cloudCentroid[0];
            iterator1.y()=cloud->points[i].y+cloudCentroid[1];
            iterator1.z()=cloud->points[i].z+cloudCentroid[2];
            octomap::OcTreeNode * node1=cloudAndUnknown.search(iterator1);	
            if(node1!=NULL){
                if (node1->getValue()==13){
                    uncoverage_points_num++;
                }
            }
        }
        candidateviewpoints_coveragenode_num[i]=cloud->points.size()-uncoverage_points_num;
        std::cout<<"the uncoverage points number is: "<< candidateviewpoints_coveragenode_num[i]<<std::endl;
        std::cout<<"---------------------------------"<<std::endl;

        // cloudAndUnknown.insertPointCloud(variablePointwall, iterator);

    }
    
    // selected the best viewpoint positions 
    float nextbest_viewpoint_position[cartesian_freedom];
    int max_coveragenode_num=0;
    int max_index;
    for (int i=0;i<candidate_viewpoints_num;i++){
        if (candidateviewpoints_coveragenode_num[i]>=max_coveragenode_num){
            max_index=i;
        }
    }
    for (int i=0; i<cartesian_freedom; i++){
        nextbest_viewpoint_position[i]=candidate_viewpoint_positions[max_index][i];
    }


    // compute the inverse kinematic solutions for next best viewpoint
    double rpy[3]={nextbest_viewpoint_position[3],nextbest_viewpoint_position[4],nextbest_viewpoint_position[5]};
    MatrixXd rot(3,3), tran(3,1), robot_matrix(4,4);
    rot=RPYtoRotMatrix(rpy);
    tran<<a[0], a[1], a[2];
    robot_matrix<<rot(0,0),rot(0,1),rot(0,2),tran(0,0),
                  rot(1,0),rot(1,1),rot(1,2),tran(1,0),
                  rot(2,0),rot(2,1),rot(2,2),tran(2,0),
                  0,0,0,1;
    std::cout<<"the robot matrix is"<<std::endl;
    std::cout<<robot_matrix<<std::endl;

    bool inverse_solution_flag;
    MatrixXd q_mat;
    inverse_solution_flag=GetInverseResult_withoutref(robot_matrix,q_mat);
    

    





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


