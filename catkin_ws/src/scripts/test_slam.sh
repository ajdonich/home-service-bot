#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e "roslaunch spherebot spherebot_spawn.launch" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch spherebot gmapping.launch" &
sleep 5

RVIZ_CONFIG=/home/workspace/home-service-bot/catkin_ws/src/rviz_config
xterm -fg lightgray -bg black -e "rosrun rviz rviz -d ${RVIZ_CONFIG}/mapping.rviz" &
sleep 5

xterm -fg lightgray -bg black -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
wait
