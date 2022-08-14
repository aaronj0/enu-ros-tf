cmake_minimum_required(VERSION 2.8.3)
project(enu_transforms)

add_compile_options(-Wall -Wextra -Wpedantic -O3 -I/usr/include/mavsdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  auvsi_msgs
)

find_package(catkin REQUIRED COMPONENTS
       actionlib
       actionlib_msgs
       geometry_msgs
       grid_map_ros
       grid_map_msgs
       nav_msgs
       std_msgs
       roscpp
       rospy
       message_generation
       grid_map_core
       grid_map_cv
       grid_map_filters
       grid_map_loader
       grid_map_octomap
       grid_map_rviz_plugin
       grid_map_visualization
       roslib
       auvsi_msgs

)
find_package(Eigen3 REQUIRED)

#add_action_files(
#        DIRECTORY action
#        FILES MissionFile.action
#)

#generate_messages(
#        DEPENDENCIES actionlib_msgs std_msgs
#)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES enu_transforms 
 CATKIN_DEPENDS roscpp rospy auvsi_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(enu_transforms src/enu_transforms/enu_transforms.cpp)

target_link_libraries(enu_transforms ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
