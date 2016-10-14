# ros_turtle

## General description

This project is a sample and introductory project to show the power of ROS and some of their capabilities. The main goal of it is to make two nodes, one path controller (server) and othe path requester (client) which will monitor and control the position of the turtle sim. 

## Nodes

The nodes are programs which run in the context of the ROS framework and interact between each othe with standard interfaces. This project involves two nodes, one for the server and the other for the client. Also, both are Python scripts and are based on the actionlib ROS library.

## Server description

The server is committed to control the turtle publishing messages on the 'turtle1/cmd_vel' topic that the turtlesim node has, and it listens to the 'turtle1/pose' topic to get the last postion and orientation of the turtle. As the turtle has a seed control, but you can measure its position, the server calculates time for rotations and traslations of the turtle and send messages to it so as to get to the goal position. Some error is made with this approach as its quite simple.

When the server receives a goal from the client, which involves the position that the client wants to send the turtle, it creates a thread that is dedicated for this control task. While the turtle is moving, some feedback is sent to the client which is composed of progress and current position. 

In case the goal has some values wrong, the server accepts the goal but sends an inmediate result with the rightPosition field in false. The client should check it so as to make sure the goal has been accepted.

## Client description

The client is a ROS node that sends position goals to the server. It is capable of track the progress, cancel, pause and resume a goal. Logic of cancel, pause and resume is handled on the client side, so the server only gets new goals while the user might have resumed a previous goal.

There is also a static web page which listens to the 'turtle1/pose' and write lines like turtlesim node. Current integration as an actionlib client is under development. It is based on ROSlibjs JavaScript library and HTML5.

## Tools & Installation

This project has been developed with:

 - Ubuntu 14.04 x64 OS
 - Sublime Text 2
 - Gnome Terminal 3.6.2
 - ROS Indigo

### ROS Indigo installation on Ubuntu 14.04

The following bash commands should be executed to get ROS Indigo installed. 

```bash
#Add the sources of the packages, keys and update repositories
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
#Full ROS Indigo installation
sudo apt-get install ros-indigo-desktop-full
sudo apt-get install ros-indigo-rosbridge-suite
#ROS Dependency manager initialization and update
sudo rosdep init
rosdep update
#ROS Environment variables
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
#ROS CLI to simply manage packages
sudo apt-get install python-rosinstall
```

The previous commands have been taken from http://wiki.ros.org/indigo/Installation/Ubuntu

### Catkin workspace initialization

If you clone this repository you should not run this, but in case you want to do things from the very beginning:

```bash
#It moves to the workspace path and then it creates a catkin workspace
cd $WORKSPACE_PATH
mkdir -p $WORKSPACE_NAME/src
cd $WORKSPACE_NAME/src
catkin_init_workspace
#It makes the empty catkin workspace
cd ..
catkin_make
source devel/setup.bash
#Create the turtle controller package.
catkin_create_pkg turtle_controller actionlib message_generation roscpp rospy std_msgs actionlib_msgs
cd ..
catkin_make
cd src
cd turtle_controller
mkdir action
#You should follow the steps described in 
#http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29 
#for generating all the action message files after the "mkdir action" command
```
After the final catkin_make, you should be able to create you Python scripts and run the code.

### To run the code

In order to run the code you should do the following steps in a console with the $WORKSPACE/devel/setup.bash script run:

```bash
# Run the ROS core node
roscore
# On a new terminal, run the node for binding the web interface to the ROS interface
roslaunch rosbridge_server rosbridge_websocket.launch
# On a new terminal, run the turtlesim. This should open a window with the turtle
rosrun turtlesim turtlesim_node
# On a new terminal, run the server
rosrun turtle_controller path_server.py
# On a new terminal, run the client
rosrun turtle_controller path_client.py
```
To see the web page, you can open with your web browser the index.html page located on the workspace/src/path_web_visualizer/src directory (relative to the repository base path).

