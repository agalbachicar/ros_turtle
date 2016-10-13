#! /usr/bin/env python

import roslib; #roslib.load_manifest('turtle_controller')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import turtle_controller.msg

#--------------------------------------------------------------------------------------------------------

# TODO! Argument check!
def getNewPositionFromCLI():
    x = raw_input("--> Please insert the new X coordinate: ")
    y = raw_input("--> Please insert the new Y coordinate: ")
    return [float(x), float(y)]


def PathClient():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('path_server', turtle_controller.msg.PathAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    while True:
        positionGoal = getNewPositionFromCLI()
        # Creates a goal to send to the action server.
        goal = turtle_controller.msg.PathGoal(positionGoal)
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        #Get the result
        result = client.get_result()
        #Check the result
        if (result.rightPosition == True):
            rospy.loginfo( "New position: [%.2f:%.2f]" % (result.currentPosition[0], result.currentPosition[1]))
        else:
            rospy.loginfo( "Error with the position sent")

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('Path_Client')
        result = PathClient()
        rospy.loginfo( "Result: %.2f" % (result.distanceAccomplished))
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")