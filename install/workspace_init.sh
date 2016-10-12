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

#It makes a package for the path requester and adds the corresponding packages to it.
cd src/ 
catkin_create_pkg path_requester std_msgs rospy

#Build catking packages
cd ..
catkin_make
. devel/setup.bash

#Create Action Server and Client Package
catkin_create_pkg turtle_controller actionlib message_generation roscpp rospy std_msgs actionlib_msgs
cd ..
catkin_make
cd src
cd turtle_controller
mkdir action





