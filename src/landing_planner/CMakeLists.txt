cmake_minimum_required(VERSION 3.0.2)
project(landing_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  tf2
  message_generation
)

find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PlanRequest.msg
  Trajectory.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs mavros_msgs roscpp tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare executables
# User input node
add_executable(user_input_node src/usr_input.cpp)
target_link_libraries(user_input_node ${catkin_LIBRARIES})
add_dependencies(user_input_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# Planner node
add_executable(landing_planner_node src/planner.cpp)
target_link_libraries(landing_planner_node ${catkin_LIBRARIES})
add_dependencies(landing_planner_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

# PX4 command sender (if needed)
add_executable(px4_cmd_sender_node src/px4_cmd_sender.cpp)
target_link_libraries(px4_cmd_sender_node ${catkin_LIBRARIES})
add_dependencies(px4_cmd_sender_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

