#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e "roslaunch spherebot spherebot_spawn.launch" &
sleep 15

xterm -fg lightgray -bg black -e "roslaunch spherebot amcl_demo.launch" &
sleep 5

RVIZ_CONFIG=/home/workspace/home-service-bot/catkin_ws/src/rviz_config
xterm -fg lightgray -bg black -e "rosrun rviz rviz -d ${RVIZ_CONFIG}/pick_objects.rviz" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch spherebot move_base.launch" &
sleep 5

xterm -fg lightgray -bg black -e "rosrun add_markers add_markers" & 
sleep 15

xterm -fg lightgray -bg black -e "rosrun pick_objects pick_objects" &
sleep 5

rosservice call /add_markers/place_marker "{action: 0, pose: {position: {x: 2.0, y: -7.0}, orientation: {w: 1.0}}}" &
wait
