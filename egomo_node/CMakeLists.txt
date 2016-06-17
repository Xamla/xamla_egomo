cmake_minimum_required(VERSION 2.8.3)
project(egomo_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   ros_egomo_node.lua
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

