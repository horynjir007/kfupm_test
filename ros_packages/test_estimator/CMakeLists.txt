cmake_minimum_required(VERSION 2.8.3)
project(kfupm_test_package)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -g -Og")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  nodelet 
  message_generation 
  std_msgs 
  std_srvs 
  tf2 
  tf2_ros 
  tf2_msgs 
  tf2_eigen 
  tf2_sensor_msgs 
  tf_conversions 
  tf2_eigen 
  tf2_geometry_msgs 
  mrs_lib)
find_package(Eigen3 REQUIRED HINTS /usr/local/lib)

# add_service_files(
#   FILES
# )
# add_message_files(
#    FILES
# )

# generate_messages(DEPENDENCIES
#   mrs_msgs
#   std_msgs
# )


catkin_package(
  # CATKIN_DEPENDS message_runtime 
  # LIBRARIES color_mapping
)
#catkin_package()

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(KfupmTest src/kfupm_test.cpp)

target_link_libraries(KfupmTest ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

