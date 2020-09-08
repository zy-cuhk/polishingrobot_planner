
#include <pluginlib/class_loader.h>
#include <ros/ros.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  const std::string node_name = "motion_planning_aubo10";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL, Start
  const std::string PLANNING_GROUP = "aubo10";
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Adding/Removing Objects and Attaching/Detaching Objects
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "map";
  collision_object.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.5;
  box_pose.position.z = 1.5;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  std::cout<<"hello"<<std::endl;
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
  // planning_scene->addCollisionDetector;

  // boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  // std::string planner_plugin_name;
  // We will get the name of planning plugin we want to load
  // if (!node_handle.getParam("planning_plugin", planner_plugin_name))
  //   ROS_FATAL_STREAM("Could not find planner plugin name");
  // try
  // {
  //   planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
  //       "moveit_core", "planning_interface::PlannerManager"));
  // }
  // catch (pluginlib::PluginlibException& ex)
  // {
  //   ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  // }
  // try
  // {
  //   planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  //   if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
  //     ROS_FATAL_STREAM("Could not initialize planner instance");
  //   ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  // }
  // catch (pluginlib::PluginlibException& ex)
  // {
  //   const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  //   std::stringstream ss;
  //   for (std::size_t i = 0; i < classes.size(); ++i)
  //     ss << classes[i] << " ";
  //   ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
  //                                                        << "Available plugins: " << ss.str());
  // }


  // Pose Goal
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  
  // geometry_msgs::PoseStamped pose;
  // pose.header.frame_id = "manipulator_base_link";
  // pose.pose.position.x = 0.3;
  // pose.pose.position.y = 0.4;
  // pose.pose.position.z = 1.75;
  // pose.pose.orientation.w = 1.0;
  // std::vector<double> tolerance_pose(3, 0.01);
  // std::vector<double> tolerance_angle(3, 0.01);

  // moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("endeffector_link", pose, tolerance_pose, tolerance_angle);
  req.group_name = PLANNING_GROUP;
  // req.goal_constraints.push_back(pose_goal);

  planning_interface::PlanningContextPtr context;
  // planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // context->solve(res);
  // if (res.error_code_.val != res.error_code_.SUCCESS)
  // {
  //   ROS_ERROR("Could not compute plan successfully");
  //   return 0;
  // }

  // publish trajectory 
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  // res.getMessage(response);
  // display_trajectory.trajectory_start = response.trajectory_start;
  // display_trajectory.trajectory.push_back(response.trajectory);
  // display_publisher.publish(display_trajectory);
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());




  // Joint Space Goals
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { 0.0,0.0,0.0,0.0,0.0, 0.0, 0.0, -1.5, -0.7, 0.0 };


  goal_state.setJointGroupPositions(joint_model_group, joint_values);

  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  req.goal_constraints.clear();

  req.goal_constraints.push_back(joint_goal);


  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    std::cout<<"hello1"<<std::endl;

  context->solve(res);


  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());


  // req.goal_constraints.clear();
  // req.goal_constraints.push_back(pose_goal);
  // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  // context->solve(res);
  // res.getMessage(response);
  // display_trajectory.trajectory.push_back(response.trajectory);
  // display_publisher.publish(display_trajectory);
  // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  // planning_scene->setCurrentState(*robot_state.get());



  return 0;
}

