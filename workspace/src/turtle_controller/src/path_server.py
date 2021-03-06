#! /usr/bin/env python

import rospy
import turtlesim.msg
from geometry_msgs.msg import Twist, Vector3
import std_msgs.msg
import math
import actionlib
import turtle_controller.msg

#Current position of the turtle
currentPose = turtlesim.msg.Pose()
#--------------------------------------------------------------------------------------------------------
class Configs:
    #Configuration arguments for the path requester
    linearSpeed = 1.0
    angularSpeed = 1.0
    distanceError = 0.1
    refreshTime = 0.5

    def __init__(self, linearSpeed = 1.0, angularSpeed = 1.0, distanceError = 0.1, refreshTime = 0.5):
        self.linearSpeed = linearSpeed
        self.angularSpeed = angularSpeed
        self.distanceError = distanceError
        self.refreshTime = refreshTime
#--------------------------------------------------------------------------------------------------------
class Publishers:
    def __init__(self, turtleName):
        self.turtleName = turtleName
        self.speedPublisher = self._initSpeedPublisher()
        # self.positionPublisher = self._initDistancePublisher()

    def _initSpeedPublisher(self):
        turtleTopic = '/' + self.turtleName + '/cmd_vel'
        publisher = rospy.Publisher(turtleTopic, Twist, queue_size=10)
        return publisher

    # def _initDistancePublisher(self):
    #     turtlePositionTopic = '/' + 'path_requester/' + self.turtleName + '/distance'
    #     publisher = rospy.Publisher(turtlePositionTopic, std_msgs.msg.Float32, queue_size=10)
    #     return publisher

    def publishSpeed(self, traslationSpeed, angularSpeed):
        self.speedPublisher.publish(Twist(Vector3(traslationSpeed, 0.0, 0.0),
                                        Vector3(0.0, 0.0, angularSpeed)))

    # def publishDistance(self, distance):
    #     self.positionPublisher.publish(distance)
#--------------------------------------------------------------------------------------------------------
class Listeners:
    def __init__(self, turtleName):
        self.turtleName = turtleName
        self.poseSuscriber = self._initPoseListener()
        # self.distanceSuscriber = self._initDistanceListener()

    def _initPoseListener(self):
        turtleTopic = '/' + self.turtleName + '/pose'
        suscriptor = rospy.Subscriber(turtleTopic, turtlesim.msg.Pose, callbackTurtlePose, turtleName)
        return suscriptor

    # def _initDistanceListener(self):
    #     turtleTopic = '/' + 'path_requester/' + self.turtleName + '/distance'
    #     suscriptor = rospy.Subscriber(turtleTopic, std_msgs.msg.Float32, callbackTurtleDistance, turtleName)
    #     return suscriptor

def callbackTurtlePose(data, turtleName):
    if (data.x != currentPose.x or 
            data.y != currentPose.y or 
            data.theta != currentPose.theta or 
            data.linear_velocity != currentPose.linear_velocity or 
            data.angular_velocity != currentPose.angular_velocity):
        #Copy current pose
        currentPose.x = data.x
        currentPose.y = data.y
        currentPose.theta = data.theta
        currentPose.linear_velocity = data.linear_velocity
        currentPose.angular_velocity = data.angular_velocity
        #Log new values
        #logCurrentPose(turtleName)       

# def callbackTurtleDistance(data, turtleName):
#     printScalar(data.data, 'Distance')

#--------------------------------------------------------------------------------------------------------
class Metrics:
    def __init__(self, posA, angleA, posB):
        self.posA = posA
        self.angleA = angleA
        self.posB = posB

    def getPositionDifference(self):
        dx = self.posB.x - self.posA.x
        dy = self.posB.y - self.posA.y
        dz = self.posB.z - self.posA.z
        return Vector3(dx, dy, dz)

    # TODO! Possible division by zero
    def getEndAngle(self,):
        delta = self.getPositionDifference()
        return math.atan2(delta.y, delta.x)

    def getAngleDifference(self, newAngle):
        cAngle = self.getAngleIn2PIModulus(self.angleA)
        nAngle = self.getAngleIn2PIModulus(newAngle)
        difAngle = self.getAngleIn2PIModulus(nAngle - cAngle)
        return difAngle

    def getAngleIn2PIModulus(self, angle):
        if angle < 0.0:
            return math.pi + math.pi + angle
        else:
            return angle

    def getDistance(self):
        diffPosition = self.getPositionDifference()
        return math.sqrt(diffPosition.x * diffPosition.x + diffPosition.y * diffPosition.y)

    # TODO! Possible division by zero
    def getRotationTime(self, newAngle, angularSpeed):
        delta = self.getAngleDifference(newAngle)
        return ( delta / angularSpeed ) 

    def getTraslationTime(self, speed):
        delta = self.getPositionDifference()
        return math.sqrt( (math.pow(delta.x, 2.0) + math.pow(delta.y, 2.0)) / speed)
#--------------------------------------------------------------------------------------------------------
# class Controller:
#     def __init__(self, publishers):
#         self._publishers = publishers

#     def _move(self, linearSpeed, angularSpeed, time, refreshTime = 0.5):
#         while (time > refreshTime):
#             self._publishers.publishSpeed(linearSpeed, angularSpeed)
#             rospy.sleep(refreshTime)
#             time = time - refreshTime

#         self._publishers.publishSpeed(linearSpeed, angularSpeed)
#         rospy.sleep(time)
#         self._publishers.publishSpeed(0.0, 0.0)

#     def _getMovementTimes(self, metrics, angularSpeed, linearSpeed):
#         positionDifference = metrics.getPositionDifference()
#         # printVector(positionDifference, 'Position difference')

#         endAngle = metrics.getEndAngle()
#         # printScalar(endAngle, 'End angle')

#         rotationTime = metrics.getRotationTime(endAngle, angularSpeed)
#         # printScalar(rotationTime, 'Rotation time')

#         linearTime = metrics.getTraslationTime(linearSpeed)
#         # printScalar(linearTime, 'Traslation time')

#         return Vector3(rotationTime, linearTime, 0.0)

#     def moveInClosedLoop(self, newPosition, distanceError, angularSpeed, linearSpeed, refreshTime = 0.5):
#         #TODO ! Possible data contention
#         currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
#         currentAngle = currentPose.theta
#         #Create the metrics object to encapsulate all the math operations
#         metrics = Metrics(currentPosition, currentAngle, newPosition)
#         #Get the current error
#         currentError = metrics.getDistance()
#         #Publish distance
#         publishers.publishDistance(currentError)

#         while (currentError > distanceError):
#             #We get the times for traslation and for rotation
#             timeVector = self._getMovementTimes(metrics, angularSpeed, linearSpeed)
#             #We make the movements
#             self._move(0.0, angularSpeed, timeVector.x, refreshTime)
#             self._move(linearSpeed, 0.0, timeVector.y, refreshTime)

#             #TODO ! Possible data contention
#             currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
#             currentAngle = currentPose.theta
#             #Create the metrics object to encapsulate all the math operations
#             metrics = Metrics(currentPosition, currentAngle, newPosition)
#             #Get the current error
#             currentError = metrics.getDistance()
#             #Publish distance
#             publishers.publishDistance(currentError)

#--------------------------------------------------------------------------------------------------------
def logCurrentPose(turtleName):
    rospy.loginfo(rospy.get_caller_id() + " %s : [x: %.2f; y: %.2f; theta: %.2f] [linear: %.2f, angular: %.2f]", 
        turtleName, currentPose.x, currentPose.y, currentPose.theta, currentPose.linear_velocity, currentPose.angular_velocity)    

def printCurrentXYAngle(turtleName):
    rospy.loginfo("--> Current %s position: [%.2f ; %.2f ]-[%.4f]", turtleName, currentPose.x, currentPose.y, currentPose.theta)

def printVector(vector, str = ''):
    rospy.loginfo("--> %s: [%.2f ; %.2f ; %.2f]", str, vector.x, vector.y, vector.z)

def printScalar(time, str = ''):
    rospy.loginfo("--> %s: [%.4f]", str, time)
#--------------------------------------------------------------------------------------------------------
class PathServerAction(object):
    # create messages that are used to publish feedback/result
    _feedback = turtle_controller.msg.PathFeedback()
    _result   = turtle_controller.msg.PathResult()

    #Position vector set as goal
    _newPosition = Vector3(0.0, 0.0, 0.0)
    #Distance to the goal
    _totalDistance = 0.0

    def __init__(self, name, publishers, configs):
        self._action_name = name
        #Set publishers and configs
        self._publishers = publishers
        self._configs = configs
        #Create the Simple Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, turtle_controller.msg.PathAction, execute_cb=self.execute_cb, auto_start = False)
        #Start the server
        self._as.start()
    
    def loadResult(self, progress, currentPosition, rightPosition):
        self._result.progress = progress
        self._result.currentPosition = [currentPosition.x, currentPosition.y]
        self._result.rightPosition = rightPosition

    #TODO Goal check for valid values
    def goalCheck(self, goal):
        self._newPosition.x = goal.newPosition[0]
        self._newPosition.y = goal.newPosition[1]
        return True

    def sendFeedback(self, currentDistance):
        #Check de distance because possible division by zero
        if (self._totalDistance == 0) :
            self._feedback.progress = 1
        else :
            self._feedback.progress = (self._totalDistance - currentDistance) / self._totalDistance
        #Set the current position
        self._feedback.currentPosition = [currentPose.x, currentPose.y]
        #Publish the feedback
        self._as.publish_feedback(self._feedback)

    def getCurrentPosition(self):
        return Vector3(currentPose.x, currentPose.y, 0.0)

    def getCurrentAngle(self):
        return currentPose.theta

    def getMovementTimes(self, metrics):
        #Get the position difference
        positionDifference = metrics.getPositionDifference()
        #Get the end angle
        endAngle = metrics.getEndAngle()
        #Calculate the rotation time
        rotationTime = metrics.getRotationTime(endAngle, self._configs.angularSpeed)
        #Calculate the traslation time
        linearTime = metrics.getTraslationTime(self._configs.linearSpeed)
        #Return time results
        return Vector3(rotationTime, linearTime, 0.0)                    

    def preemptionOccured(self):
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            return True
        return False

    def move(self, linearSpeed, angularSpeed, time):
        while (time > self._configs.refreshTime):
            # Check that preempt has not been requested by the client
            if (self.preemptionOccured() == True):
                return False

            #Send turtle signal of speed and time
            self._publishers.publishSpeed(linearSpeed, angularSpeed)
            rospy.sleep(self._configs.refreshTime)
            time = time - self._configs.refreshTime
            #Create the metrics object to encapsulate to give feedback of the position
            metrics = Metrics(self.getCurrentPosition(), self.getCurrentAngle(), self._newPosition)
            self.sendFeedback(metrics.getDistance())

        #Send remanent speed information
        self._publishers.publishSpeed(linearSpeed, angularSpeed)
        rospy.sleep(time)
        #Stop
        self._publishers.publishSpeed(0.0, 0.0)
        #Create the metrics object to encapsulate to give feedback of the position
        metrics = Metrics(self.getCurrentPosition(), self.getCurrentAngle(), self._newPosition)
        self.sendFeedback(metrics.getDistance())  

        return True

    def executeController(self):
        #TODO ! Possible data contention
        #Create the metrics object to encapsulate all the math operations
        metrics = Metrics(self.getCurrentPosition(), self.getCurrentAngle(), self._newPosition)
        #Set the distance to the goal
        self._totalDistance = metrics.getDistance()
        #Calculate the movement times
        timeVector = self.getMovementTimes(metrics)

        #Calculate and send the feedback
        self.sendFeedback(self._totalDistance)        

        #Rotation movement
        if(self.move(0.0, self._configs.angularSpeed, timeVector.x) == False):
            return False

        #Traslation movement
        if(self.move(self._configs.linearSpeed, 0.0, timeVector.y) == False):
            return False

        return True


    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        #Set the current progress
        self._feedback.progress = 0.0

        if (self.goalCheck(goal) == False):
            rospy.loginfo('%s: Goal validation error.' % self._action_name)
            #Load de result of the action goal
            loadResult(self._feedback.progress, currentPose, False)
            #Send result to client
            self._as.set_succeeded(self._result)
            #End action task
            return

        # Publish info to the console for the user
        rospy.loginfo('%s: Controlling turtle to move from here to  [%.2f;%.2f]' % (self._action_name, self._newPosition.x, self._newPosition.y))

        # Action starts
        if (self.executeController() == False):
            #Action fails due to preemption
            return

        #Action ends
        self._feedback.progress = 1.0
        self.loadResult(self._feedback.progress, self.getCurrentPosition(), True)
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
    rospy.init_node('path_server')
    turtleName = 'turtle1'
    #Load the configurations
    configs = Configs()
    #Initialize the turtle position listener
    listener = Listeners(turtleName)
    #Initialize the publishers
    publishers = Publishers(turtleName)


    rospy.loginfo('Path Server initiated.')
    rospy.loginfo('Listening for incoming goals...')

    PathServerAction(rospy.get_name(), publishers, configs)
    rospy.spin()
