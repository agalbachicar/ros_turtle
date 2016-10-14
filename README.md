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

## Tools & Installation

This project has been developed with:

 - Ubuntu 14.04 x64 OS
 - Sublime Text 2
 - Gnome Terminal 3.6.2
 - ROS Indigo

To 