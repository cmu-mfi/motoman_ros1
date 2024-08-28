cmake_minimum_required(VERSION 3.0.2)
project(yk_tasks)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  motoman_gp4_moveit_config
  moveit_ros_planning_interface
  moveit_ros_move_group
  tf
  roscpp
  geometry_msgs
  std_msgs
  message_generation
  moveit_msgs
  actionlib
  actionlib_msgs
  yk_msgs
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES 
  GoToJoints.action
  GoToPose.action
  ExecuteCartesianTrajectory.action
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetPose.srv
  GetPoseStamped.srv
  SetPose.srv
  SetJoints.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  moveit_msgs
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS 
     motoman_gp4_moveit_config 
     roscpp
     tf
     message_runtime
     geometry_msgs
     moveit_msgs
     std_msgs
     actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(yk_tasks src/main.cpp src/yk_interface.cpp)
add_executable(service_sample_node src/service_sample_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(yk_tasks ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(service_sample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(yk_tasks
  ${catkin_LIBRARIES}
)
target_link_libraries(service_sample_node
  ${catkin_LIBRARIES}
)
