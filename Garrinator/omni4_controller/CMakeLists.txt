cmake_minimum_required(VERSION 2.8.3)
project(omni4_controller)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	controller_interface
	realtime_tools
	urdf
	eigen_conversions
	control_msgs
	geometry_msgs
	pluginlib
)

catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		moby_controllers
	CATKIN_DEPENDS
		roscpp
		controller_interface
		realtime_tools
		urdf
		eigen_conversions
		geometry_msgs
		pluginlib
)

include_directories(
	include
  	${catkin_INCLUDE_DIRS}
)

## Build
add_library(${PROJECT_NAME}
	src/omni4_controller.cpp
)
add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)
