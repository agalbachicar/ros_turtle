#!/bin/bash

#Follow instructions from: http://wiki.ros.org/indigo/Installation/Ubuntu

#Add the sources of the packages, keys and update repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update

#Full ROS Indigo installation
sudo apt-get install ros-indigo-desktop-full

#To find available packages use the following
# apt-cache search ros-indigo

#ROS Dependency manager initialization and update
sudo rosdep init
rosdep update

#ROS Environment variables
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

#ROS CLI to sumply manage packages
sudo apt-get install python-rosinstall

