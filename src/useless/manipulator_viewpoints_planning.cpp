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
	ros::init (argc, argv, "viewpoint_planning_using_octomap");  
	ros::NodeHandle nh;  
    int hz=10;
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
    octomap::OcTree cloudAndUnknown(0.01), cloudAndUnknown1(0.01), cloudAndUnknown2(0.01);
    for (size_t i = 0; i < cloud.points.size (); ++i){ 
        octomap::OcTreeNode * cloudNode=cloudAndUnknown.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
        cloudNode->setValue(1);

        octomap::OcTreeNode * cloudNode1=cloudAndUnknown1.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
        cloudNode1->setValue(1);

        octomap::OcTreeNode * cloudNode2=cloudAndUnknown2.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
        cloudNode2->setValue(1);
    }

    // phase 1-step3: create octree of camera FOV 
    octomap::point3d Point3dwall(0.0,0.0,1.20);    
    octomap::Pointcloud pointwall;     
    for(int ii=1;ii<283;ii++){
        for(int iii=1;iii<173;iii++){
            Point3dwall.y()= (-0.4920)+(ii*0.003489);
            Point3dwall.x()= (-0.3080)+(iii*0.003574);
            pointwall.push_back(Point3dwall);
        }
    }


    // phase 1-step4: sample candidate camera viewpoint positions 
    int candidate_viewpoints_num = 8;
    int cartesian_freedom = 6;
    float candidate_viewpoint_positions[candidate_viewpoints_num][cartesian_freedom]=
    {
        {-0.2,   0.35,  0.2,  0.0, M_PI/2, 0.0},
        {-0.2,  -0.35,  0.2,  0.0, M_PI/2, 0.0},
        { 0.2,  0.2, -0.4,  0.0, 3*M_PI/4, 0.0},
        { 0.2, -0.2, -0.4,  0.0, 3*M_PI/4, 0.0},
        { 0.2,  0.3,  0.3,  0.0, 3*M_PI/4, 0.0},
        { 0.2, -0.3,  0.3,  0.0, 3*M_PI/4, 0.0},
        {-0.2,  0.0,  0.7,  0.0,  M_PI/2,  0.0},
        {-0.2,  0.0,  1.25, 0.0,  M_PI/2,  0.0},
    }; 
    float candidate_viewpoint_flag[candidate_viewpoints_num];
    for (int i=0; i<candidate_viewpoints_num; i++){
        candidate_viewpoint_flag[i]=1;
    }

    float manipulatorbase_position[6]={0.18, 0.0, 1.196, 0.0, 0.0, 0.0};
    int candidateviewpoints_coveragenode_num[candidate_viewpoints_num];
    
    // phase 1-step5: initialize the final output: onecellviewpoints_candidatejointsolutions_dict
    int selected_viewpoint=0;
    std::string str1, str2; 
    Json::Value onecellviewpoints_candidatejointsolutions_dict;
    Json::Value onecellviewpoints_position_dict;
    Json::StyledWriter swriter;



    //-----------------------------------------------------------------------------------------------------------------------------------
    // phase 2: the NBV algorithm for coverage viewpoint planning problem 
    while(ros::ok()){
        // phase 2-step1: obtain covered octomap nodes number for candidate viewpoints using octree cloudAndUnknown
        for (int i=0; i<candidate_viewpoints_num; i++){
            // phase 2-step 1.1: obtain the pose of camera viewpoint 
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

            // phase 2-step 1.2: raycast to obtain voxel from the camera viewpoint pose
            octomap::KeyRay rayBeam;
            int unknownVoxelsInRay=0;
            int known_points_projection=0;
            for (int ii=0; ii<variablePointwall.size();ii++){
                bool Continue=true;		
                cloudAndUnknown1.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
                for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                    octomap::OcTreeNode * node=cloudAndUnknown1.search(*it);	
                    if(node!=NULL){
                        cloudAndUnknown1.updateNode(*it, false);
                        Continue=false;
                    }
                }
            }
            cloudAndUnknown1.updateInnerOccupancy();

            // phase 2-step 1.3: obtian covered voxel numbers for candidate camera viewpoint poses 
            int uncoverage_points_num=0;
            
            for (size_t i = 0; i < cloud.points.size (); ++i){ 
                octomap::point3d iterator1; 
                iterator1.x()=cloud.points[i].x+cloudCentroid[0];
                iterator1.y()=cloud.points[i].y+cloudCentroid[1];
                iterator1.z()=cloud.points[i].z+cloudCentroid[2];
                octomap::OcTreeNode * node1=cloudAndUnknown1.search(iterator1);	
                if(node1!=NULL){
                    if (node1->getValue()==1){
                        uncoverage_points_num++;
                    }
                }
            }
            candidateviewpoints_coveragenode_num[i]=cloud.points.size()-uncoverage_points_num;
            cout<<"the index of candidate viewpoint is: "<<i<<endl;
            std::cout<<"the coverage points number for candidate viewpoint is: "<< candidateviewpoints_coveragenode_num[i]<<std::endl;
            // cloudAndUnknown.insertPointCloud(variablePointwall, iterator);

            // phase 2-step 1.4: reset octree cloudAndUnknown1 from cloudAndUnknown for each loop
            for (size_t i = 0; i < cloud.points.size (); ++i){ 
                octomap::point3d iterator1; 
                iterator1.x()=cloud.points[i].x+cloudCentroid[0];
                iterator1.y()=cloud.points[i].y+cloudCentroid[1];
                iterator1.z()=cloud.points[i].z+cloudCentroid[2];
                octomap::OcTreeNode * node1=cloudAndUnknown.search(iterator1);	
                if (node1->getValue()==1){
                    octomap::OcTreeNode * cloudNode=cloudAndUnknown1.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
                    cloudNode->setValue(1);
                }
                else{
                    octomap::OcTreeNode * cloudNode=cloudAndUnknown1.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],false);
                }
            }
            
        }
        // phase 2-step 2: selected the best viewpoint positions through the comparision of covered voxel numbers 
        float possible_nextbest_viewpoint_position[cartesian_freedom];
        int max_coveragenode_num=0;
        int max_index;
        for (int i=0;i<candidate_viewpoints_num;i++){
            cout<<"the flag for candidate viewpoint is:"<<candidate_viewpoint_flag[i]<<endl;
            if ((candidate_viewpoint_flag[i]!=0)&&(candidateviewpoints_coveragenode_num[i]>=max_coveragenode_num)){
                max_index=i;
                max_coveragenode_num=candidateviewpoints_coveragenode_num[i];
                cout<<"the max index is:"<<max_index<<endl;
            }
        }
        for (int i=0; i<cartesian_freedom; i++){
            possible_nextbest_viewpoint_position[i]=candidate_viewpoint_positions[max_index][i];
        }

        // phase 2-step 3: compute the inverse kinematic solutions for next best viewpoint
        double rpy[3]={possible_nextbest_viewpoint_position[3],possible_nextbest_viewpoint_position[4],possible_nextbest_viewpoint_position[5]};
        MatrixXd rot(3,3), tran(3,1), robot_matrix(4,4);
        rot=RPYtoRotMatrix(rpy);
        tran<<possible_nextbest_viewpoint_position[0], possible_nextbest_viewpoint_position[1], possible_nextbest_viewpoint_position[2];
        robot_matrix<<rot(0,0),rot(0,1),rot(0,2),tran(0,0),
                    rot(1,0),rot(1,1),rot(1,2),tran(1,0),
                    rot(2,0),rot(2,1),rot(2,2),tran(2,0),
                    0,0,0,1;

        bool inverse_solution_flag;
        MatrixXd q_mat;
        VectorXd aubo_q(6);
        inverse_solution_flag=GetInverseResult_withoutref(robot_matrix,q_mat);
        // phase 2-step4: select collision-free joint solutions for selected viewpoint position
        float pub_joints[6];
        bool judge_self_collision_flag;

        if (inverse_solution_flag==true){
            int qsolution_num=int(q_mat.size()/6);
            bool viewpoint_collision_state=1;
            bool qsolutions_collisionstates[qsolution_num]={0};

            for (int i=0;i<qsolution_num;i++){
                aubo_q<<q_mat(0,i),q_mat(1,i),q_mat(2,i),q_mat(3,i),q_mat(4,i),q_mat(5,i);
                // cout<<"aubo q is:"<<aubo_q<<endl;
                for (size_t j=0; j<6;j++){
                    pub_joints[j]=aubo_q(j);
                }
                while (ros::ok()) {
                    if ((pub_joints[0]==sub_joints[0])&&(pub_joints[1]==sub_joints[1])&&(pub_joints[2]==sub_joints[2])&&(pub_joints[3]==sub_joints[3])&&(pub_joints[4]==sub_joints[4])&&(pub_joints[5]==sub_joints[5])){
                        ros::param::get("/judge_self_collision_flag", judge_self_collision_flag);
                            if (judge_self_collision_flag==0){
                                qsolutions_collisionstates[i]=0;
                                // cout<<"aubo qlist: "<<pub_joints[0]<<" "<<pub_joints[1]<<" "<<pub_joints[2]<<" "<<pub_joints[3]<<" "<<pub_joints[4]<<" "<<pub_joints[5]<<endl;
                                cout<<"no collision"<<endl;
                                // cout<<"--------------------------------------------------";
                            }
                            else{
                                qsolutions_collisionstates[i]=1;
                                // cout<<"aubo qlist: "<<pub_joints[0]<<" "<<pub_joints[1]<<" "<<pub_joints[2]<<" "<<pub_joints[3]<<" "<<pub_joints[4]<<" "<<pub_joints[5]<<endl;
                                cout<<"collision"<<endl;
                                // cout<<"--------------------------------------------------";
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
            }
            // judge whether existing collision-free solutions 
            for (int j=0;j<qsolution_num;j++){
                viewpoint_collision_state=viewpoint_collision_state & qsolutions_collisionstates[j];
            }
            // output collision free solutions to the dict
            int collisionfree_qsolution_num=0;
            if (viewpoint_collision_state==0){
                str1=to_string(selected_viewpoint)+"th_selected_viewpoint";
                for (int k=0;k<qsolution_num;k++){
                    aubo_q<<q_mat(0,k),q_mat(1,k),q_mat(2,k),q_mat(3,k),q_mat(4,k),q_mat(5,k);
                    for (size_t j=0; j<6;j++){
                        pub_joints[j]=aubo_q(j);
                    }
                    if (qsolutions_collisionstates[k]==0){
                        str2=to_string(collisionfree_qsolution_num)+"th_candidate_joint_solution";
                        for (int m=0;m<6;m++){
                            onecellviewpoints_candidatejointsolutions_dict[str1][str2][m]=pub_joints[m];
                        }
                        collisionfree_qsolution_num+=1;
                    }
                }
            }

            // output collision free cartesian-space solution to the list
            if (viewpoint_collision_state==0){
                std::string str3=to_string(selected_viewpoint)+"th_selected_viewpointposition";
                for (int i=0; i<6; i++){
                    onecellviewpoints_position_dict[str3][i]=possible_nextbest_viewpoint_position[i];
                }
                // change the coverage state of octree for wall cloud points 
                octomap::Pointcloud variablePointwall;     
                octomap::point3d iterator; 
                iterator.x()=possible_nextbest_viewpoint_position[0]+manipulatorbase_position[0];        
                iterator.y()=possible_nextbest_viewpoint_position[1]+manipulatorbase_position[1];    
                iterator.z()=possible_nextbest_viewpoint_position[2]+manipulatorbase_position[2];    
                
                float roll, pitch, yaw;
                roll=possible_nextbest_viewpoint_position[3]+manipulatorbase_position[3];  
                pitch=possible_nextbest_viewpoint_position[4]+manipulatorbase_position[4];  
                yaw=possible_nextbest_viewpoint_position[5]+manipulatorbase_position[5];  

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
                    cloudAndUnknown.computeRayKeys(iterator,variablePointwall.getPoint(ii),rayBeam);
                    for(octomap::KeyRay::iterator it=rayBeam.begin(); it!=rayBeam.end() && Continue; it++){
                        octomap::OcTreeNode * node=cloudAndUnknown.search(*it);	
                        if(node!=NULL){
                            cloudAndUnknown.updateNode(*it, false);
                            Continue=false;
                        }
                    }
                }
                cloudAndUnknown.updateInnerOccupancy();

                // copy operation for octree cloudAndUnknown1 from cloudAndUnknown
                int real_covered_num=0;
                for (size_t i = 0; i < cloud.points.size (); ++i){ 
                    octomap::point3d iterator1; 
                    iterator1.x()=cloud.points[i].x+cloudCentroid[0];
                    iterator1.y()=cloud.points[i].y+cloudCentroid[1];
                    iterator1.z()=cloud.points[i].z+cloudCentroid[2];
                    octomap::OcTreeNode * node1=cloudAndUnknown.search(iterator1);	
                    if (node1->getValue()==1){
                            octomap::OcTreeNode * cloudNode=cloudAndUnknown1.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],true);
                            cloudNode->setValue(1);
                        }
                    else{
                        octomap::OcTreeNode * cloudNode=cloudAndUnknown1.updateNode(cloud.points[i].x+cloudCentroid[0],cloud.points[i].y+cloudCentroid[1],cloud.points[i].z+cloudCentroid[2],false);
                        real_covered_num+=1;
                    }
                }
                cout<<"the viewpoint position is:"<<possible_nextbest_viewpoint_position[0]<<" "<<possible_nextbest_viewpoint_position[1]<<" "<<possible_nextbest_viewpoint_position[2]<<" "<<possible_nextbest_viewpoint_position[3]<<" "<<possible_nextbest_viewpoint_position[4]<<" "<<possible_nextbest_viewpoint_position[5]<<endl;
                cout<<"the total_covered_num is: "<<cloud.points.size ()<<endl;
                cout<<"real_covered_num for effective viewpoint is: "<<real_covered_num<<endl;
                // delete the chosed viewpoint from cnadidate viewpoint list
                candidate_viewpoint_flag[max_index]=0;
                selected_viewpoint+=1;
            }
        }
        cout<<"the candidate viewpoint flag is: "<<candidate_viewpoint_flag[0]<<endl; //<<" "<<candidate_viewpoint_flag[1]<<endl;
        // phase 2-step 5: the exit condition is shown as follows:
        if (selected_viewpoint==candidate_viewpoints_num){
            break;
        }
    }


    //--------------------------------------------------------------------------------------------------------------------------------------------
    // phase 3: select joint solutions from candidate joint solutions 
    // phase 3-step1 : save candidate joint solutions of viewpoint positions into onecellviewpoints_candidatejointsolutions_dict
    float aubo_q1[6];
    for (int i=0; i<onecellviewpoints_candidatejointsolutions_dict.size(); i++){
        str1=to_string(i)+"th_selected_viewpoint";
        for (int j=0; j<onecellviewpoints_candidatejointsolutions_dict[str1].size(); j++){
            str2=to_string(j)+"th_candidate_joint_solution";
            for (int k=0; k<onecellviewpoints_candidatejointsolutions_dict[str1][str2].size(); k++){
                aubo_q1[k] = onecellviewpoints_candidatejointsolutions_dict[str1][str2][k].asFloat();
            }
            // cout<<"aubo q is: "<<aubo_q1[0]<<" "<<aubo_q1[1]<<" "<<aubo_q1[2]<<" "<<aubo_q1[3]<<" "<<aubo_q1[4]<<" "<<aubo_q1[5]<<endl;
        }
    }
    std::ofstream ofs("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/src/onecellviewpoints_candidatejointsolutions_dict.json");
    ofs << onecellviewpoints_candidatejointsolutions_dict;
    ofs.close();

    // phase 3-step2 : save cartesian-space positions of viewpoint into onecellviewpoints_position_dict
    float aubo_viewposition[6];
    for (int i=0;i<onecellviewpoints_position_dict.size();i++){
        std::string str3=to_string(i)+"th_selected_viewpointposition";
        for (int j=0;j<6;j++){
            aubo_viewposition[j]=onecellviewpoints_position_dict[str3][j].asFloat();
        }
        // cout<<"aubo position is: "<<aubo_viewposition[0]<<" "<<aubo_viewposition[1]<<" "<<aubo_viewposition[2]<<" "<<aubo_viewposition[3]<<" "<<aubo_viewposition[4]<<" "<<aubo_viewposition[5]<<endl;
    }
    std::ofstream ofs1("/home/zy/catkin_ws/src/polishingrobot_ylz/polishingrobot_planner/src/onecellviewpoints_position_dict.json");
    ofs1 << onecellviewpoints_position_dict;
    ofs.close();


    
    return 0;

}


