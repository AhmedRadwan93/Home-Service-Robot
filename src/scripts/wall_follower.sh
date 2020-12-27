#!/bin/bash



# Launch the nodes
xterm  -e "export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/world/ahmed__world.world"; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 10
xterm  -e "roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:="$(pwd)/src/slam_gmapping/gmapping/launch/slam_gmapping_pr2.launch"  " &
sleep 5
xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e "rosrun wall_follower wall_follower"
