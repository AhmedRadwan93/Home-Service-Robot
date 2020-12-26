cmake_minimum_required(VERSION 2.8.3)
project(add_markers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  nav_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES add_markers
#  CATKIN_DEPENDS roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/add_markers.cpp
# )

add_executable(add_markers src/add_markers.cpp)


target_link_libraries(add_markers ${catkin_LIBRARIES})

