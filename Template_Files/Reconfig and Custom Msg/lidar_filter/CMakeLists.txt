cmake_minimum_required(VERSION 2.8.3)
project(lidar_filter)


add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  tf2_geometry_msgs
  tf2_ros
  dynamic_reconfigure
  message_generation
  std_msgs
  geometry_msgs
  jsk_recognition_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   cluster_info.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/PointCloudFilter.cfg
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy message_runtime
)

# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

# Compile an executable node
add_executable(lidar_filter 
  src/lidar_filter.cpp
  src/main.cpp)

add_dependencies(lidar_filter
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
  
target_link_libraries(lidar_filter ${catkin_LIBRARIES})
