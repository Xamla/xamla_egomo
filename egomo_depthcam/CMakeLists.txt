cmake_minimum_required(VERSION 2.8.3)
project(egomo_depthcam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS camera_info_manager dynamic_reconfigure image_transport roscpp std_msgs std_srvs sensor_msgs message_generation)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(PkgConfig)

pkg_check_modules(PC_OPENNI2 libopenni2)
if (NOT PC_OPENNI2_FOUND)
  pkg_check_modules(PC_OPENNI2 REQUIRED openni2)
endif()

pkg_check_modules(PC_PNG libpng>=1.2.50)
if (NOT PC_PNG_FOUND)
  pkg_check_modules(PC_PNG REQUIRED png>=1.2.50)
endif()

#pkg_check_modules(PC_LZ4 liblz4>=129)
#if (NOT PC_LZ4_FOUND)
#  pkg_check_modules(PC_LZ4 REQUIRED lz4>=129)
#endif()
# TODO: Replace by a correct package finding routine
set(LIBLZ4_LIBRARIES "/usr/local/lib/liblz4.so" )
set(LIBLZ4_INCLUDE_DIRS "/usr/local/include" )


################################################
## add dynamic reconfigure api
################################################

#generate_dynamic_reconfigure_options(cfg/OpenNI2.cfg)


################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
   FILES
   GetNewImage.srv
   SetParameters.srv
   GetSerial.srv
 )

add_message_files(
  FILES
  DepthImage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS camera_info_manager dynamic_reconfigure image_transport sensor_msgs roscpp message_runtime
  DEPENDS libopenni2 libpng liblz4
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PC_OPENNI2_INCLUDE_DIRS}
  ${LIBLZ4_INCLUDE_DIRS}
)

link_directories(${PC_OPENNI2_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
   src/openni2_device.cpp
   src/openni2_exception.cpp
   src/openni2_listener.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PC_OPENNI2_LIBRARIES} ${Boost_LIBRARIES} ${PC_PNG_LIBRARIES} ${LIBLZ4_LIBRARIES})


add_executable(egomo_depthcam_node
   nodes/egomo_depthcam_node.cpp
)
target_link_libraries(egomo_depthcam_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PC_PNG_LIBRARIES} ${LIBLZ4_LIBRARIES})
add_dependencies(egomo_depthcam_node ${PROJECT_NAME}_generate_messages_cpp)


#############
## Install ##
#############

## Copy launch files
##install(DIRECTORY launch/
##  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
##  FILES_MATCHING PATTERN "*.launch"
##)

install(TARGETS ${PROJECT_NAME} egomo_depthcam_node ${ADDITIONAL_EXECUTABLES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
