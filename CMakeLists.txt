cmake_minimum_required(VERSION 2.8.3)
project(drones)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  std_msgs
  message_generation
  mav_msgs
  nav_msgs
  eigen_conversions
  formation_control_lib
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  EstimatedDronePosition.msg
  EstimatedDronePositionArray.msg
  BearingVectors.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime sensor_msgs std_msgs tf eigen_conversions
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_include_dirs}
  ${eigen3_include_dirs}
)

add_executable(animation_rviz_node src/animation_rviz_node.cpp src/animation_rviz.cpp)
target_link_libraries(animation_rviz_node ${catkin_LIBRARIES})
add_dependencies(animation_rviz_node ${animation_rviz_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(drone_operator src/drone_operator_node.cpp src/outerloop_controller.cpp src/command_creator.cpp)
target_link_libraries(drone_operator ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(drone_operator ${drone_operator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
