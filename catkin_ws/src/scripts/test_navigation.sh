#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e "roslaunch spherebot spherebot_spawn.launch" &
sleep 10

xterm -fg lightgray -bg black -e "roslaunch spherebot amcl_demo.launch" &
sleep 10

RVIZ_CONFIG=/home/workspace/home-service-bot/catkin_ws/src/rviz_config
xterm -fg lightgray -bg black -e "rosrun rviz rviz -d ${RVIZ_CONFIG}/localization.rviz" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch spherebot move_base.launch"
sleep 15
wait
