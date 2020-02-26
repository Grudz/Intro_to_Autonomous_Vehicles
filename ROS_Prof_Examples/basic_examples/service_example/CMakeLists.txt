cmake_minimum_required(VERSION 2.8.3)

project(service_example)

# List other catkin package dependencies
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf
    std_msgs
    message_generation
)

# Include core ROS library directories
include_directories(${catkin_INCLUDE_DIRS})

#Messages should be placed here
add_service_files(FILES Adder.srv)

generate_messages()

# Declare catkin package
catkin_package()

# Compile an executable node
add_executable(service_advertiser src/service_advertiser.cpp)
add_dependencies(service_advertiser ${PROJECT_NAME}_gencpp)
target_link_libraries(service_advertiser ${catkin_LIBRARIES})

add_executable(service_client src/service_client.cpp)
add_dependencies(service_client ${PROJECT_NAME}_gencpp)
target_link_libraries(service_client ${catkin_LIBRARIES})
