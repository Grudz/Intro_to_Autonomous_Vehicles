cmake_minimum_required(VERSION 2.8.3)
project(homework3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)



add_executable(diff_drive src/diff_drive.cpp)
target_link_libraries(diff_drive
  ${catkin_LIBRARIES}
)

add_executable(mecanum_drive src/mecanum_drive.cpp)
target_link_libraries(mecanum_drive
  ${catkin_LIBRARIES}
)

add_executable(ackermann src/ackermann.cpp)
target_link_libraries(ackermann
  ${catkin_LIBRARIES}
)

add_executable(test_diff test/test_diff.cpp)
target_link_libraries(test_diff
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_executable(test_mecanum test/test_mecanum.cpp)
target_link_libraries(test_mecanum
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_executable(test_ackermann test/test_ackermann.cpp)
target_link_libraries(test_ackermann
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

