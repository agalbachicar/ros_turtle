#! /usr/bin/env python

import roslib; #roslib.load_manifest('turtle_controller')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import turtle_controller.msg

import sys
from select import select


#--------------------------------------------------------------------------------------------------------

# TODO! Argument check!
def getNewPositionFromCLI():
    print('------------------------------------------------')
    x = raw_input("--> Please insert the new X coordinate: ")
    y = raw_input("--> Please insert the new Y coordinate: ")
    return [float(x), float(y)]

def scanConsoleWithTimeout(text, timeoutTime):
    print("%s" % (text))
    rlist, _, _ = select([sys.stdin], [], [], timeoutTime)
    if rlist:
        s = sys.stdin.readline()
        return s
    else:
        return None

def clearOutput():
    print("\033c")
    return

#--------------------------------------------------------------------------------------------------------
class PathClient:

    def __init__(self, serverName):
        # Creates the SimpleActionClient, passing the type of the action
        self.client = actionlib.SimpleActionClient(serverName, turtle_controller.msg.PathAction)
        self.terminalState = None
        self.feedbackValue = None
        self.resultValue = None
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
        self.terminalState = terminalState
        self.resultValue = resultValue   
        return

    def activeCallback(self):
        #rospy.loginfo("Goal sent to active state")
        return

    def feedbackCallback(self, feedbackValue):
        self.feedbackValue = feedbackValue
        # rospy.loginfo("Current progress: %.2f %%" % (feedbackValue.progress * 100.0))
        # rospy.loginfo("Current position: [%.2f:%.2f]" % (feedbackValue.currentPosition[0], feedbackValue.currentPosition[1]))
        return 

    def getCurrentProgress(self):
        if(self.feedbackValue != None):
            return self.feedbackValue.progress
        return 0.0

    def getCurrentPosition(self):
        if(self.feedbackValue != None):
            return self.feedbackValue.currentPosition
        return None

    def getIsGoalFinished(self):
        if(self.terminalState == None):
            return False
        return True

    def getTerminalState(self):
        return self.terminalState

    def getResultValue(self):
        return self.resultValue

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

def createTextProgress(progress, currentPosition):
    if (currentPosition != None):
        return 'Progress: ' + str(progress*100.0) + ' % | Position: [' + str(currentPosition[0]) + ' ; ' + str(currentPosition[1]) + ']'
    else:
        return 'Progress: ' + str(progress)

def createEndGoalText(terminalState, progress, currentPosition):
    if (terminalState != None and progress != None != currentPosition != None):
        return 'Terminal state: ' + str(terminalState) + 'Progress: ' + str(progress*100.0) + ' % | Position: [' + str(currentPosition[0]) + ' ; ' + str(currentPosition[1]) + ']'
    elif (terminalState != None):
        return 'Terminal state: ' + str(terminalState)
    else:
        return ''

def clientAsyncController(positionGoal):
    pathClient = PathClient('path_server')
    pathClient.connectToServer()
    pathClient.sendGoal(positionGoal, 0.0)

    while (pathClient.getIsGoalFinished() == False):

        progress = pathClient.getCurrentProgress()
        currentPosition = pathClient.getCurrentPosition()

        textProgress = createTextProgress(progress, currentPosition)

        clearOutput()        

        print ('%s' % (textProgress))
        input = scanConsoleWithTimeout('Press c to cancel, p to pause and r to resume', 1)

        if (input == None):
            continue
        elif (input == 'p'):
            print ('Pause!!!!!')
        elif (input == 'c'):
            print ('Cancel!!!!!')
        elif (input == 'r'):
            print ('Resume!!!!!')
        else:
            continue

    textResult = createEndGoalText(pathClient.getTerminalState(), pathClient.getResultValue().progress, pathClient.getResultValue().currentPosition)
    print(textResult)
    
    return    

if __name__ == '__main__':

    rospy.init_node('Path_Client', anonymous=True)
    try:
        # clientBlockingController()
        while True:
            positionGoal = getNewPositionFromCLI()
            clientAsyncController(positionGoal)
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