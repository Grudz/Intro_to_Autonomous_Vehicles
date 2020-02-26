cmake_minimum_required(VERSION 2.8.3)

project(gps_example)

# List other catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    ugv_course_libs
)

# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(gps_example src/gps_example.cpp)
target_link_libraries(gps_example ${catkin_LIBRARIES})

