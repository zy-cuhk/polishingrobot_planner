cmake_minimum_required(VERSION 2.8.3)
project(polishingrobot_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  octomap_msgs
	rospy
	std_msgs
	rostime 
	sensor_msgs 
	message_filters 
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OCTOMAP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
    octomap_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include 

)

add_executable(collision_detection src/moveit_collision_detection.cpp)
target_link_libraries(collision_detection ${catkin_LIBRARIES})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(manipulator_viewpoints_planning src/manipulator_viewpoints_planning.cpp src/aubo_kinematics.cpp)
target_link_libraries(manipulator_viewpoints_planning ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(mobileplatform_planningresult_visualization src/mobileplatform_planningresult_visualization.cpp)
target_link_libraries(mobileplatform_planningresult_visualization ${catkin_LIBRARIES} )


# the below cpp documents are not suitable 
add_executable(polishingrobot_states_pub src/polishingrobot_states_pub.cpp)
target_link_libraries(polishingrobot_states_pub ${catkin_LIBRARIES} )

add_executable(polishingrobot_states_sub src/polishingrobot_states_sub.cpp)
target_link_libraries(polishingrobot_states_sub ${catkin_LIBRARIES} )

add_executable(polishingrobot_states_pubsub src/polishingrobot_states_pubsub.cpp)
target_link_libraries(polishingrobot_states_pubsub ${catkin_LIBRARIES} )

add_executable(polishingrobot_statespub_collisiondetection src/polishingrobot_statespub_collisiondetection.cpp)
target_link_libraries(polishingrobot_statespub_collisiondetection ${catkin_LIBRARIES} )