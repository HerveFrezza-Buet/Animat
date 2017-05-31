cmake_minimum_required(VERSION 2.8.3)
project(color_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  message_generation
)

find_package(OpenCV)

generate_dynamic_reconfigure_options(
  cfg/param.cfg
)

add_message_files(
  FILES
  HSVParams.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  image_transport
  cv_bridge
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
)

add_executable        (color-filter-hsv-node src/color-filter-hsv-node.cc)
add_dependencies      (color-filter-hsv-node color_filter_gencpp)
target_link_libraries (color-filter-hsv-node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})   

