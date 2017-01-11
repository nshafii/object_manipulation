cmake_minimum_required(VERSION 2.8.3)
project(unknown_object_grasping)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	tf
	pcl_ros
    	tf2_ros
	geometry_msgs
	sensor_msgs	
	std_msgs
	pluginlib 
	cmake_modules
	genmsg
    message_generation
	INESC_Robotis_Driver
)

#find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(FILES GraspingNearestObject.srv)

generate_messages(DEPENDENCIES std_msgs)

#Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES unknown_object_grasping
  CATKIN_DEPENDS message_runtime
 )

#add_library(INESC_Robotis_Driver
#src/IK_Solver.cpp
#src/GoToPose_service.cpp
#src/TrajectoryGenerator.cpp)

#target_link_libraries(INESC_Robotis_Driver ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(unknown_object_grasping src/unknown_object_grasping.cpp)
target_link_libraries(unknown_object_grasping ${catkin_LIBRARIES})
add_dependencies(unknown_object_grasping unknown_object_grasping_gencpp)




## Specify libraries to link a library or executable target against


