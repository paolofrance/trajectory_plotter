cmake_minimum_required(VERSION 3.0.2)
project(trajectory_plotter)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  roscpp
  std_msgs
  rosdyn_core
  moveit_core
  moveit_ros_planning_interface
  tf_conversions
  moveit_planning_helper
  )

add_service_files(
   FILES
   ioc.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
  rosdyn_core
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)


add_library(
human_traj_estimation src/human_traj_estimation.cpp
)

add_executable(human_traj_estimation_node src/human_traj_estimation_node.cpp)
target_link_libraries( human_traj_estimation_node human_traj_estimation ${catkin_LIBRARIES} fuzzylite)
add_dependencies(human_traj_estimation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(robot_traj_generator src/robot_traj_generator.cpp)
target_link_libraries( robot_traj_generator ${catkin_LIBRARIES} )
add_dependencies(robot_traj_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




