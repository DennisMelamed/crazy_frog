cmake_minimum_required(VERSION 2.8.3)
project(crazyFrog)

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
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(laserCalibration src/laserCalibration.cpp)
add_dependencies(odom_listen line_driver laserCalibration nav_msgs_generate_messages_cpp realRoomRendezvous realRoomRendezvous1 roomRendezvous roomRendezvous1 rotationTest amcl_pose_readerdriver bug2 bug2new createRendezvous createRendezvous1 gmapping_pos)
target_link_libraries(line_driver ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
