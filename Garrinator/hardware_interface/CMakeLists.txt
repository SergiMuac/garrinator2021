cmake_minimum_required(VERSION 2.8.3)
project(garrinator_hardware_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	controller_manager
	hardware_interface
	combined_robot_hw
	joint_limits_interface
)

catkin_package(
	CATKIN_DEPENDS
		roscpp
		controller_manager
		hardware_interface
		combined_robot_hw
		joint_limits_interface)


include_directories(include
	${catkin_INCLUDE_DIRS} )


###########
## Build ##
###########

add_library(${PROJECT_NAME}
	src/odrive_hw_if.cpp
)
add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

add_executable(hw_if_node
	src/hw_if_node.cpp
)
add_dependencies(hw_if_node
	${PROJECT_NAME}
)
target_link_libraries(hw_if_node
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)
