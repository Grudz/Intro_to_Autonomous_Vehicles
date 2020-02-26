cmake_minimum_required(VERSION 2.8.3)

project(tf_example)

# List other catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf
    std_msgs
    dynamic_reconfigure
    message_generation
)

# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

generate_messages()

generate_dynamic_reconfigure_options(cfg/TFExample.cfg)

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(tf_example src/tf_example.cpp)
add_dependencies(tf_example ${PROJECT_NAME}_gencfg)
target_link_libraries(tf_example ${catkin_LIBRARIES})

