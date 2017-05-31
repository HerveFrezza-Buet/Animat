cmake_minimum_required(VERSION 2.8.3)
project(animat)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS 
  scripts/vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
