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

def createTextProgress(progress, currentPosition):
    if (currentPosition != None):
        return 'Progress: ' + str(progress*100.0) + ' % | Position: [' + str(currentPosition[0]) + ' ; ' + str(currentPosition[1]) + ']'
    else:
        return 'Progress: ' + str(progress)

def createEndGoalText(terminalState, progress, currentPosition):
    if (terminalState != None and progress != None != currentPosition != None):
        return 'Terminal state: ' + str(terminalState) + ' | Progress: ' + str(progress*100.0) + ' % | Position: [' + str(currentPosition[0]) + ' ; ' + str(currentPosition[1]) + ']'
    elif (terminalState != None):
        return 'Terminal state: ' + str(terminalState)
    else:
        return ''

def createCancelGoalText():
    return 'Goal has been cancelled'

def resumeCurrentGoal():
    while rospy.is_shutdown() == False:
        input = raw_input('Press "r" to resume or "c" to cancel:')

        if(input.startswith('r') or input.startswith('R')):
            return True
        elif(input.startswith('c') or input.startswith('C')):
            return False
        else:
            print('Bad response...')
    return False

def sendNewGoal():            
    while rospy.is_shutdown() == False:
        input = raw_input('Do you want to send a goal? [y/n]:')

        if(input.startswith('y') or input.startswith('Y')):
            return True
        elif(input.startswith('n') or input.startswith('N')):
            return False
        else:
            print('Bad response...')        
    return False
#--------------------------------------------------------------------------------------------------------
class PathClient:

    def __init__(self, serverName):
        # Creates the SimpleActionClient, passing the type of the action
        self.client = actionlib.SimpleActionClient(serverName, turtle_controller.msg.PathAction)
        self.terminalState = None
        self.feedbackValue = None
        self.resultValue = None
        return

    def connectToServer(self, timeoutTime = 10):
        # Waits until the action server has started up and started
        # listening for goals.        
        return self.client.wait_for_server(rospy.Duration(timeoutTime, 0))

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

    def cancelGoal(self):
        self.client.cancel_goal()
        return

    def resetGoal(self):
        self.goal = None
        self.terminalState = None
        self.feedbackValue = None
        self.resultValue = None
        return

    def doneCallback(self, terminalState, resultValue):
        self.terminalState = terminalState
        self.resultValue = resultValue   
        return

    def activeCallback(self):
        #rospy.loginfo("Goal sent to active state")
        return

    def feedbackCallback(self, feedbackValue):
        self.feedbackValue = feedbackValue
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

#--------------------------------------------------------------------------------------------------------
def clientAsyncController(positionGoal):
    #Create a client to talk to the server
    pathClient = PathClient('path_server')
    #Connect to the server
    if( pathClient.connectToServer() == False):
        print ('Server connection timeout')
        return False

    #Send the goal the server
    pathClient.sendGoal(positionGoal, 0.0)

    #Iterate while the goal is being processed
    while (pathClient.getIsGoalFinished() == False and rospy.is_shutdown() == False):
        #Get the current progress
        progress = pathClient.getCurrentProgress()
        #Get the current position
        currentPosition = pathClient.getCurrentPosition()
        #Generate a legend with those values
        textProgress = createTextProgress(progress, currentPosition)
        #Clear output to refresh data
        clearOutput()        
        #Print progress and ask for the action to take
        print ('%s' % (textProgress))
        input = scanConsoleWithTimeout('Press "p" to pause, "c" to cancel... ', 2)

        #Parse response and act in consecuence
        if (input == None):
            continue
        elif (input.startswith('c') or input.startswith('C')):
            #We need to cancel the goal so we just end the task
            pathClient.cancelGoal()
            print('%s' % (createCancelGoalText()))
            return True
        elif (input.startswith('p') or input.startswith('P')):
            #Cancel current goal
            pathClient.cancelGoal()
            #Ask if the client wants to resume current goal
            if (resumeCurrentGoal() == True):
                pathClient.resetGoal()
                pathClient.sendGoal(positionGoal, 0.0)
            else:
                print('%s' % (createCancelGoalText()))   
                return True

    textResult = createEndGoalText(pathClient.getTerminalState(), pathClient.getResultValue().progress, pathClient.getResultValue().currentPosition)
    print(textResult)

    return True

if __name__ == '__main__':

    print('Path_Client is starting...')
    
    rospy.init_node('Path_Client', anonymous=True)

    try:
        while (sendNewGoal() == True and rospy.is_shutdown() == False) :
            #Ask for the goal
            positionGoal = getNewPositionFromCLI()
            #Run the async client controller
            clientAsyncController(positionGoal)
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")

    print('Path_Client is finishing...')
