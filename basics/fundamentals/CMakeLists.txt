cmake_minimum_required(VERSION 2.8.3)
project(basics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(fundamentals 
  src/fundamentals.cpp
  src/main.cpp)
  
target_link_libraries(fundamentals ${catkin_LIBRARIES})
