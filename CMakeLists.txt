cmake_minimum_required(VERSION 2.8.3)
project(crazy_frog)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf
  message_generation
  roslib
)

add_message_files(
	FILES
	MacroRequest.msg
	CompilerData.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp roslib
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(runtime src/runtime.cpp ${catkin_LIBRARIES})
target_link_libraries(runtime ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
