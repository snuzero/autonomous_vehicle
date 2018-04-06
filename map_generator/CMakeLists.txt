cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  core_msgs
  geometry_msgs
  image_transport
  message_filters
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES map_generator
  CATKIN_DEPENDS core_msgs geometry_msgs image_transport message_filters roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/map_generator.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(map_gen src/map_generator.cpp)
add_dependencies(map_gen core_msgs_generate_messages_cpp)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (cv_bridge REQUIRED)

## Specify libraries to link a library or executable target against
target_link_libraries(map_gen
   ${catkin_LIBRARIES} ${Eigen3_LIBS} ${OpenCV_LIBS} ${cv_bridge_LIBRARIES} ${PROJECT_NAME}
)
