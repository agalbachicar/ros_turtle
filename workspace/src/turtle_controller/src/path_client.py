#! /usr/bin/env python

import roslib; #roslib.load_manifest('turtle_controller')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import turtle_controller.msg

def PathClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('path_server', turtle_controller.msg.PathAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = turtle_controller.msg.PathGoal(distanceAccomplished=1.0)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Path_Client')
        result = PathClient()
        rospy.loginfo( "Result: %.2f" % (result.distanceAccomplished))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")