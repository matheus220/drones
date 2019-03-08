cmake_minimum_required(VERSION 2.8.3)
project(drones)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  std_msgs
  image_transport
  cv_bridge
  message_generation
  mav_msgs
  nav_msgs
  eigen_conversions
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
  CATKIN_DEPENDS roscpp message_runtime sensor_msgs std_msgs cv_bridge image_transport tf eigen_conversions
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

add_executable(broadcastTransformsBase src/broadcastTransformsBase.cpp)
target_link_libraries(broadcastTransformsBase ${catkin_LIBRARIES})

add_executable(addMarkers src/addMarkers.cpp)
target_link_libraries(addMarkers ${catkin_LIBRARIES})
add_dependencies(addMarkers ${addMarkers_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(drone_operator src/drone_operator_node.cpp src/target_detector.cpp src/outerloop_controller.cpp src/command_creator.cpp)
target_link_libraries(drone_operator ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(drone_operator ${drone_operator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})