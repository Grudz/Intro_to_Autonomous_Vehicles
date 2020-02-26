cmake_minimum_required(VERSION 2.8.3)

project(diff_drive_kinematics)

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
add_executable(diff_drive_kinematics src/diff_drive_kinematics.cpp)
target_link_libraries(diff_drive_kinematics ${catkin_LIBRARIES})
