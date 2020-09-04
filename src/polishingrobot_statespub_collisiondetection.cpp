#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


void add_objects( moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

  Eigen::Vector3d world_size;
  world_size << 1,1,1;
  shapes::Mesh* load_mesh = shapes::createMeshFromResource("package://polishingrobot_planner/mesh/20200716_scan_planning_2.stl", world_size);  
  if(load_mesh==NULL) 
  {
      ROS_WARN("mesh is NULL !!!");
      return;
  } 
  
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(load_mesh, mesh_msg);
  shape_msgs::Mesh mesh;
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  geometry_msgs::Pose pose;
  pose.orientation.w =1.0;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 1;

  moveit_msgs::CollisionObject obj;
  obj.header.frame_id = "map";
  obj.id="dae_mesh";
  obj.mesh_poses.push_back(pose);
  obj.meshes.push_back(mesh);
  obj.operation = obj.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(obj);
  planning_scene_interface.addCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Publisher joint_pub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);

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


  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
  new robot_model_loader::RobotModelLoader("robot_description"));

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startStateMonitor();

  if(planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startSceneMonitor("/planning_scene");
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();
  }
  else
  {
    ROS_ERROR("Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();
  robot_state::RobotState& current_state = planning_scene->getCurrentStateNonConst();
  add_objects(planning_scene_interface);
  //sleep(2);

  ros::Rate loop_rate(10.0);
  bool self_collision_state, environment_collision_state;

  while (ros::ok())
  {
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

      bool exist_dae_mesh = planning_scene->getWorld()->hasObject("dae_mesh");
      ROS_WARN("exist_dae_mesh: %d", exist_dae_mesh);
      current_state.update();

      ROS_WARN("printStatePositions:");
      current_state.printStatePositions();

      // self collision check 
      collision_result.clear();
      planning_scene->checkSelfCollision(collision_request, collision_result, current_state);
      ROS_INFO_STREAM("Test SelfCollision Check  : Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
      //std::cout<<"collision_result.collision "<< collision_result.collision<<std::endl;
      if (collision_result.collision==1){
          self_collision_state=1;
      }
      else{
          self_collision_state=0;
      }

      // environment collision check  
      collision_result.clear();
      collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
      planning_scene->checkCollision(collision_request, collision_result, current_state, acm);
      ROS_INFO_STREAM("Test Collision With Environment Check  : Current state is " << (collision_result.collision ? "in" : "not in") << " collision with environment");
      if (collision_result.collision==1){
          environment_collision_state=1;
      }
      else{
          environment_collision_state=0;
      }
      
      // collisiion state publish
      if(ros::param::has("/judge_self_collision_flag"))
      {
          collision_result.collision = self_collision_state|environment_collision_state;
          ros::param::set("/judge_self_collision_flag",collision_result.collision);
      }
      loop_rate.sleep();
  }


  ros::shutdown();
  return 0;
}
