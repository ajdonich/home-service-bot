#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e "roslaunch spherebot spherebot_spawn.launch" &
sleep 10

xterm -fg lightgray -bg black -e "roslaunch spherebot amcl_demo.launch"
sleep 15
wait
