cmake_minimum_required(VERSION 2.8.3)

project(transform_example)

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
add_executable(transform_example1 src/transform_example1.cpp)
target_link_libraries(transform_example1 ${catkin_LIBRARIES})

add_executable(transform_example2 src/transform_example2.cpp)
target_link_libraries(transform_example2 ${catkin_LIBRARIES})

add_executable(tf_example1 src/tf_example1.cpp)
target_link_libraries(tf_example1 ${catkin_LIBRARIES})

add_executable(tf_example2 src/tf_example2.cpp)
target_link_libraries(tf_example2 ${catkin_LIBRARIES})

add_executable(tf_example2_pubsub src/tf_example2_pubsub.cpp)
target_link_libraries(tf_example2_pubsub ${catkin_LIBRARIES})

