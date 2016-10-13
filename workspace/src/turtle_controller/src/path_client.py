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

#--------------------------------------------------------------------------------------------------------
class PathClient:

    def __init__(self, serverName):
        # Creates the SimpleActionClient, passing the type of the action
        self.client = actionlib.SimpleActionClient(serverName, turtle_controller.msg.PathAction)
        return

    def connectToServer(self):
        # Waits until the action server has started up and started
        # listening for goals.        
        self.client.wait_for_server()
        return

    def sendGoal(self, positionGoal, blockTime):
        # Creates a goal to send to the action server.
        self.goal = turtle_controller.msg.PathGoal(positionGoal)

        if(blockTime > 0):
            # Sends the goal to the action server.
            self.client.send_goal(self.goal)
            # Waits for the server to finish performing the action.
            self.client.wait_for_result()
            #Return the result
            return self.client.get_result()
        else:
            # Sends the goal to the action server.
            self.client.send_goal(self.goal,
                                self.doneCallback,
                                self.activeCallback,
                                self.feedbackCallback)
        return None

    def doneCallback(self, terminalState, resultValue):
        #Check the result
        rospy.loginfo("Terminal state: %d" % (terminalState))

        if (resultValue.rightPosition == True):
            rospy.loginfo( "Progress result: %.2f %%" % (resultValue.progress * 100.0))
            rospy.loginfo( "New position: [%.2f:%.2f]" % (resultValue.currentPosition[0], resultValue.currentPosition[1]))
        else:
            rospy.loginfo("Error with the position sent")        

        return

    def activeCallback(self):
        rospy.loginfo("Goal sent to terminal state")
        return

    def feedbackCallback(self, feedbackValue):
        rospy.loginfo("Current progress: %.2f %%" % (feedbackValue.progress * 100.0))
        rospy.loginfo("Current position: [%.2f:%.2f]" % (feedbackValue.currentPosition[0], feedbackValue.currentPosition[1]))
        return 

def clientBlockingController():
    while True:
        positionGoal = getNewPositionFromCLI()

        pathClient = PathClient('path_server')
        pathClient.connectToServer()
        
        result = pathClient.sendGoal(positionGoal, 30.0)
        # Check the result
        if (result.rightPosition == True):
            rospy.loginfo( "New position: [%.2f:%.2f]" % (result.currentPosition[0], result.currentPosition[1]))
            rospy.loginfo( "Progress result: %.2f" % (result.progress))
        else:
            rospy.loginfo( "Error with the position sent")

    return

def clientAsyncController():
    # while True:
    positionGoal = getNewPositionFromCLI()

    pathClient = PathClient('path_server')
    pathClient.connectToServer()
    pathClient.sendGoal(positionGoal, 0.0)

    rospy.spin()

        # # Check the result
        # if (result.rightPosition == True):
        #     rospy.loginfo( "New position: [%.2f:%.2f]" % (result.currentPosition[0], result.currentPosition[1]))
        #     rospy.loginfo( "Progress result: %.2f" % (result.progress))
        # else:
        #     rospy.loginfo( "Error with the position sent")

    return    

if __name__ == '__main__':

    rospy.init_node('Path_Client', anonymous=True)
    try:
        # clientBlockingController()
        clientAsyncController()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")
    # try:
    #     # Initializes a rospy node so that the SimpleActionClient can
    #     # publish and subscribe over ROS.
    #     rospy.init_node('Path_Client', anonymous=True)
    #     result = PathClient()
    #     rospy.loginfo( "Result: %.2f" % (result.distanceAccomplished))
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Program interrupted before completion")