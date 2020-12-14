#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e "roslaunch spherebot spherebot_spawn.launch" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch spherebot gmapping.launch" &
sleep 5

xterm -fg lightgray -bg black -e "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
sleep 15
wait
