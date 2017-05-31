cmake_minimum_required(VERSION 2.8.3)
project(focus)

find_package(catkin REQUIRED
  roscpp 
  std_msgs 
  image_transport
  cv_bridge
  dynamic_reconfigure)

find_package(OpenCV)

find_package(PkgConfig)
pkg_check_modules(FFTW fftw3 REQUIRED)


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

add_library(neuralfield 
  SHARED
  src/convolution_fftw.cc src/neuralfield.cc)
target_link_libraries (neuralfield ${FFTW_LIBRARIES})
target_compile_options(neuralfield PUBLIC ${FFTW_CFLAGS_OTHER} -std=c++11)
target_include_directories(neuralfield PUBLIC ${FFTW_INCLUDE_DIRS})


add_executable(focus-node src/focus-node.cc)
add_dependencies(focus-node neuralfield)
target_link_libraries(focus-node ${catkin_LIBRARIES} neuralfield)

