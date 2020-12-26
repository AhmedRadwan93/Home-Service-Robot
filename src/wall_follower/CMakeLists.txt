cmake_minimum_required(VERSION 2.8.3)
project(wall_follower)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    std_msgs
    actionlib
)



catkin_package(

)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(wall_follower src/wall_follower.cpp)


target_link_libraries(wall_follower ${catkin_LIBRARIES})




