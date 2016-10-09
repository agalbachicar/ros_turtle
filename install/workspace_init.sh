#!/bin/bash
#Inspired from http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

#Variables
WORKSPACE_PATH=..
WORKSPACE_NAME=workspace

#It moves to the workspace path and then it creates a catkin workspace
cd $WORKSPACE_PATH
mkdir -p $WORKSPACE_NAME/src
cd $WORKSPACE_NAME/src
catkin_init_workspace

#It makes the empty catkin workspace
cd ..
catkin_make
source devel/setup.bash




