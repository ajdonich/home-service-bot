#!/bin/sh

# Launch
xterm -fg lightgray -bg black -e "roslaunch spherebot spherebot_spawn.launch" &
sleep 10

xterm -fg lightgray -bg black -e "roslaunch spherebot amcl_demo.launch" &
sleep 10

RVIZ_CONFIG=/home/workspace/home-service-bot/catkin_ws/src/rviz_config
xterm -fg lightgray -bg black -e "rosrun rviz rviz -d ${RVIZ_CONFIG}/pick_objects.rviz" &
sleep 5

xterm -fg lightgray -bg black -e "roslaunch spherebot move_base.launch" &
sleep 5

xterm -fg lightgray -bg black -e "rosrun add_markers add_markers"
sleep 5

#xterm -fg lightgray -bg black -e "rosrun pick_objects pick_objects"
wait
