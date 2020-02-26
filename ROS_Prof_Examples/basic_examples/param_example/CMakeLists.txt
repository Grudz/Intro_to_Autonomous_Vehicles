cmake_minimum_required(VERSION 2.8.3)

project(param_example)

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

generate_dynamic_reconfigure_options(cfg/ReconfigExample.cfg)

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(static_param_example src/static_param_example.cpp)
target_link_libraries(static_param_example ${catkin_LIBRARIES})

add_executable(dynamic_reconfig_example src/dynamic_reconfig_example.cpp)
add_dependencies(dynamic_reconfig_example ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_reconfig_example ${catkin_LIBRARIES})
