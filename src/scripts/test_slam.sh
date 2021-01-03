#!/bin/bash



# Launch the nodes
xterm  -e "roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(rospack find add_markers)/world/ahmed__world.world" &

sleep 10  

xterm  -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5

xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm  -e "roslaunch  roslaunch turtlebot_teleop keyboard_teleop.launch"



