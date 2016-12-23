cmake_minimum_required(VERSION 2.8.3)
project(preprocessing)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	tf
    	tf2_ros
	geometry_msgs
	sensor_msgs	
	std_msgs
	pluginlib 
	cmake_modules
	genmsg
    message_generation
)

#find_package(Boost REQUIRED system filesystem date_time thread)

#Build talker and listener
include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES INESC_Robotis_Driver
#  CATKIN_DEPENDS message_runtime
# )

#add_library(INESC_Robotis_Driver
#src/IK_Solver.cpp
#src/GoToPose_service.cpp
#src/TrajectoryGenerator.cpp)

#target_link_libraries(INESC_Robotis_Driver ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(preprocessing src/preprocessing.cpp)
target_link_libraries(preprocessing ${catkin_LIBRARIES})
add_dependencies(preprocessing preprocessing_gencpp)




## Specify libraries to link a library or executable target against


