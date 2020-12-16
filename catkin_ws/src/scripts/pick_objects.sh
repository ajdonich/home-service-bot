#!/bin/sh

# Launch
rosservice call /add_markers/place_marker "{action: 2}" &
sleep 5 

rosservice call /add_markers/place_marker "{action: 0, pose: \
    {position: {x: 2.0, y: -7.0}, orientation: {w: 1.0}}}" &
wait
