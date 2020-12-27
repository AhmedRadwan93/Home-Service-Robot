#!/bin/bash



# Launch the nodes
xterm  -e "export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/world/ahmed__world.world"; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 7
xterm  -e "export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/world/map.yaml"; roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e "rosrun add_markers add_markers"
