cmake_minimum_required(VERSION 2.8.3)
project(laser_matcher)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_msgs   
  geometry_msgs
  tf
  tf2
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

add_executable(laser_matcher src/laser_matcher.cpp src/eigen_laserm_2d.cpp)
target_link_libraries(laser_matcher ${catkin_LIBRARIES})
#add_dependencies(laser_matcher laser_matcher)
