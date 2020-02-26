cmake_minimum_required(VERSION 2.8.3)

project(time_example)

# List other catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf
    std_msgs
)

# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(time_example src/time_example.cpp)
target_link_libraries(time_example ${catkin_LIBRARIES})
