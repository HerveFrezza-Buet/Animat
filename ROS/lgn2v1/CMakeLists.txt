cmake_minimum_required(VERSION 2.8.3)
project(lgn2v1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  message_generation
  cv_bridge
  dynamic_reconfigure
)

find_package(OpenCV)

generate_dynamic_reconfigure_options(
  cfg/param.cfg
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
  ${catkin_INCLUDE_DIRS}
)

add_executable        (lgn2v1-node src/lgn2v1-node.cc)
add_dependencies      (lgn2v1-node lgn2v1_gencfg)
target_link_libraries (lgn2v1-node ${catkin_LIBRARIES})   
target_compile_options(lgn2v1-node PUBLIC -std=c++11)
set_property          (TARGET lgn2v1-node APPEND PROPERTY INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
set_property          (TARGET lgn2v1-node APPEND PROPERTY INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
set_property          (TARGET lgn2v1-node APPEND PROPERTY LINK_LIBRARIES      ${OpenCV_LIBRARIES})

