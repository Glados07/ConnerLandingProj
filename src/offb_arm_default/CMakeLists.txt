cmake_minimum_required(VERSION 3.0.2)
project(offb_arm_default)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(offb_arm_init src/offb_arm_init.cpp)
target_link_libraries(offb_arm_init ${catkin_LIBRARIES})
