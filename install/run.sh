#!/bin/bash

cd ..
roscore &
roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun turtlesim turtlesim_node