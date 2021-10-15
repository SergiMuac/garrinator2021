cmake_minimum_required(VERSION 2.8.3)
project(garri_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
)
catkin_package(

)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(robot_teleop_joy src/robot_teleop_joy.cpp)
target_link_libraries(robot_teleop_joy ${catkin_LIBRARIES})
