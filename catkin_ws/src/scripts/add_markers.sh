#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5

xterm -fg lightgray -bg black -e "rosrun add_markers add_markers" &
sleep 5

RVIZ_CONFIG=/home/workspace/home-service-bot/catkin_ws/src/rviz_config
xterm -fg lightgray -bg black -e "rosrun rviz rviz -d ${RVIZ_CONFIG}/add_markers.rviz" &
sleep 5

rosservice call /add_markers/place_marker "{action: 0, pose: {position: {x: 4.0, y: -4.0}, orientation: {w: 1.0}}}"
sleep 5

rosservice call /add_markers/place_marker "{action: 2}"
sleep 5

rosservice call /add_markers/place_marker "{action: 0, pose: {position: {x: -4.0, y: 4.0}, orientation: {w: 1.0}}}"
wait