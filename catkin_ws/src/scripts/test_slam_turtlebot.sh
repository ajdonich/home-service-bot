#!/bin/sh

# Script environment vars
GAZEBO_WORLD=/home/workspace/home-service-bot/catkin_ws/src/map/home_service.world

# Launch
xterm -fg lightgray -bg black -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${GAZEBO_WORLD}" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"

