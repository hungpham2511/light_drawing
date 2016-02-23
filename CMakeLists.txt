cmake_minimum_required(VERSION 2.8.3)
project(light_drawing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  denso_openrave
  printx_controller
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  DrawingRequest.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS denso_openrave printx_controller rospy std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
