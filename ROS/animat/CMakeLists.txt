cmake_minimum_required(VERSION 2.8.3)
project(animat)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/triangle_contro.cfg
)


catkin_package(
  CATKIN_DEPENDS 
  rospy
  std_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
)


catkin_install_python(
  PROGRAMS 
  scripts/triangle_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
