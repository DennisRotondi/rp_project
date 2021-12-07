cmake_minimum_required(VERSION 3.0.2)
project(laser_matcher)

set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  . /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_executable(laser_matcher src/laser_matcher.cpp src/eigen_icp_2d.cpp)
target_link_libraries(laser_matcher ${catkin_LIBRARIES})
add_dependencies(laser_matcher laser_matcher)
