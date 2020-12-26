#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/world/ahmed__world.world " &
sleep 7
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch  map_file:=/home/workspace/catkin_ws/src/world/map.yaml " &
sleep 7
xterm -e " rosrun rviz rviz -d ~/catkin_ws/src/rvizConfig/home_service.rviz " &
sleep 10
xterm -e "rosrun add_markers add_markers " &
sleep 7
xterm -e "rosrun pick_objects pick_objects "





  
