cmake_minimum_required(VERSION 3.0.2)
project(com3528_team8)

find_package(catkin REQUIRED COMPONENTS
cv_bridge
geometry_msgs
rospy
sensor_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)