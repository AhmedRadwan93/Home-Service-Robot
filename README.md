# Home Service Robot


# Cloned Packages
The home service robot project uses the below ROS packages, cloned all of them:

[gmapping](http://wiki.ros.org/gmapping)

[turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)

[turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)

[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)

# Installing the pkgs
$ git clone https://github.com/ros-perception/slam_gmapping

$ git clone https://github.com/turtlebot/turtlebot

$ git clone https://github.com/turtlebot/turtlebot_interactions

$ git clone https://github.com/turtlebot/turtlebot_simulator


# Package Map:

    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file
    │   ├── turtlebot_gazebo
    │   ├── ...
    |
    |
    ├── World                          # world files
    │   ├── ...
    ├── ShellScripts                   # shell scripts files
    │   ├── ...
    ├──RvizConfig                      # rviz configuration files
    │   ├── ...
    ├──wall_follower                   # wall_follower C++ node
    │   ├── src/wall_follower.cpp
    │   ├── ...
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── world/ahmed__world.world   # world files
    └──
```
# Launch files:

Create launch.sh file with the following content

#!/bin/sh
xterm  -e  " gazebo " &
sleep 5
xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm  -e  " rosrun rviz rviz"

Run launch.sh to test Gazebo and RViz are working.


## Testing SLAM

$ roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(find rospack add_markers/world/ahmed__world.world)

Run test_slam.sh to test slam is working, it requires manual movement of the robot.


## Wall Follower

Run wall_follower.sh to autonomously navigate and create map.

# Test Navigation

$ roslaunch turtlebot_gazebo amcl_demo.launch map_file:=~/catkin_ws/src/worlds/umap.yaml


# Pick Objects


Run pick_objects.sh to run two step navigation.


# Add Markers


Run add_markers.sh to add and remove a marker on the pick up and drop off locations.


# Home Service

Run home_service.sh to pick and drop marker to specified locations.


