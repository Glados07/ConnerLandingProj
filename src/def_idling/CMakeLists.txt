cmake_minimum_required(VERSION 3.0.2)
project(def_idling)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(default_idle src/default_idle.cpp)
target_link_libraries(default_idle ${catkin_LIBRARIES})