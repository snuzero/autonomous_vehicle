cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
)

add_message_files( DIRECTORY msg
  FILES
  ImagePoint.msg
  CompressedImagePoint.msg
  ROIPointArray.msg
  Vector3DArray.msg
  MapFlag.msg
  XboxFlag.msg
  LaneInfo.msg
)

## Generate services in the 'srv' folder
 add_service_files( DIRECTORY srv
   FILES
   CameraImg.srv
   MapGenPoint.srv
   CameraCrop.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES   geometry_msgs  sensor_msgs   std_msgs )

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs std_msgs)
