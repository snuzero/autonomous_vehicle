cmake_minimum_required(VERSION 2.8.3)
project(lms_client)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  std_msgs
  core_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(lms_client src/lms_client.cpp)
add_dependencies(lms_client core_msgs_generate_messages_cpp)
target_link_libraries(lms_client ${catkin_LIBRARIES})
