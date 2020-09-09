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

# the useful algorithm is shown as follows

add_executable(collision_detection src/moveit_collision_detection.cpp)
target_link_libraries(collision_detection ${catkin_LIBRARIES})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(manipulator_viewpoints_planning1 src/manipulator_viewpoints_planning1.cpp src/aubo_kinematics.cpp)
target_link_libraries(manipulator_viewpoints_planning1 ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(manipulator_viewpoints_visualizing src/manipulator_viewpoints_visualizing.cpp)
target_link_libraries(manipulator_viewpoints_visualizing ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(manipulator_viewpoints_trajectory_planning src/manipulator_viewpoints_trajectory_planning.cpp)
target_link_libraries(manipulator_viewpoints_trajectory_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES} jsoncpp)

add_executable(framework3d_visualizing src/framework3d_visualizing.cpp)
target_link_libraries(framework3d_visualizing ${catkin_LIBRARIES} )

# the below cpp document is for test coverage state visualization method in RVIZ
add_executable(octomap_occupancymapgrid_test src/coveragestate_visualization/octomap_occupancymapgrid_test.cpp)
target_link_libraries(octomap_occupancymapgrid_test ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(nav_gridcells_test  src/coveragestate_visualization/nav_gridcells_test.cpp)
add_dependencies(nav_gridcells_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_gridcells_test ${catkin_LIBRARIES} )

add_executable(nav_occupancygrid_test  src/coveragestate_visualization/nav_occupancygrid_test.cpp)
add_dependencies(nav_occupancygrid_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_occupancygrid_test ${catkin_LIBRARIES} )

# the below moveit motion planning algorithm is not useful: 
add_executable(manipulator_viewpoints_trajectory_planning1 src/useless/manipulator_viewpoints_trajectory_planning1.cpp)
target_link_libraries(manipulator_viewpoints_trajectory_planning1 ${catkin_LIBRARIES} ${Boost_LIBRARIES} jsoncpp)

add_executable(motion_planning_aubo src/useless/motion_planning_aubo.cpp)
target_link_libraries(motion_planning_aubo ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(manipulator_viewpoints_planning src/useless/manipulator_viewpoints_planning.cpp src/aubo_kinematics.cpp)
target_link_libraries(manipulator_viewpoints_planning ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(polishingrobot_states_pub src/useless/polishingrobot_states_pub.cpp)
target_link_libraries(polishingrobot_states_pub ${catkin_LIBRARIES} )

add_executable(polishingrobot_states_sub src/useless/polishingrobot_states_sub.cpp)
target_link_libraries(polishingrobot_states_sub ${catkin_LIBRARIES} )

add_executable(polishingrobot_states_pubsub src/useless/polishingrobot_states_pubsub.cpp)
target_link_libraries(polishingrobot_states_pubsub ${catkin_LIBRARIES} )

add_executable(polishingrobot_statespub_collisiondetection src/useless/polishingrobot_statespub_collisiondetection.cpp)
target_link_libraries(polishingrobot_statespub_collisiondetection ${catkin_LIBRARIES} )

add_executable(move_group_aubo_tutorial src/useless/move_group_aubo_tutorial.cpp)
target_link_libraries(move_group_aubo_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})