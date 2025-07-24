cmake_minimum_required(VERSION 3.0.2)
project(lidar_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/lidar_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)