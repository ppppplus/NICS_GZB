cmake_minimum_required(VERSION 2.8.3)
project(nicsrobot_line_follower)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs dynamic_reconfigure message_runtime)
find_package(OpenCV REQUIRED)



## Declare ROS messages and services
#add_message_files(FILES pos.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


catkin_package( CATKIN_DEPENDS roscpp rospy)

## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


catkin_install_python(PROGRAMS scripts/level_1/detection_node.py scripts/level_1/motion_node.py
scripts/level_2/detection_node_2.py scripts/level_2/motion_node_2.py
scripts/help/control_node.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


