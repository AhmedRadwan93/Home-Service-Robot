#!/bin/bash


# Launch the nodes
xterm  -e "roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(rospack find add_markers)/world/ahmed__world.world" &

sleep 7

xterm  -e "roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=$(rospack find add_markers)/world/map.yaml" &


sleep 5

xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 10

xterm  -e "rosrun add_markers add_markers" &

sleep 5

xterm  -e "rosrun pick_objects pick_objects"






  
