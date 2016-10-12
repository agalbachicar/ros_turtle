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
    refreshTime = 0.01

    def __init__(self, linearSpeed = 1.0, angularSpeed = 1.0, distanceError = 0.1, refreshTime = 0.01):
        self.linearSpeed = linearSpeed
        self.angularSpeed = angularSpeed
        self.distanceError = distanceError
        self.refreshTime = refreshTime
#--------------------------------------------------------------------------------------------------------
class Publishers:
    def __init__(self, turtleName):
        self.turtleName = turtleName
        self.speedPublisher = self._initSpeedPublisher()
        self.positionPublisher = self._initDistancePublisher()

    def _initSpeedPublisher(self):
        turtleTopic = '/' + self.turtleName + '/cmd_vel'
        publisher = rospy.Publisher(turtleTopic, Twist, queue_size=10)
        return publisher

    def _initDistancePublisher(self):
        turtlePositionTopic = '/' + 'path_requester/' + self.turtleName + '/distance'
        publisher = rospy.Publisher(turtlePositionTopic, std_msgs.msg.Float32, queue_size=10)
        return publisher

    def publishSpeed(self, traslationSpeed, angularSpeed):
        self.speedPublisher.publish(Twist(Vector3(traslationSpeed, 0.0, 0.0),
                                        Vector3(0.0, 0.0, angularSpeed)))

    def publishDistance(self, distance):
        self.positionPublisher.publish(distance)
#--------------------------------------------------------------------------------------------------------
class Listeners:
    def __init__(self, turtleName):
        self.turtleName = turtleName
        self.poseSuscriber = self._initPoseListener()
        self.distanceSuscriber = self._initDistanceListener()

    def _initPoseListener(self):
        turtleTopic = '/' + self.turtleName + '/pose'
        suscriptor = rospy.Subscriber(turtleTopic, turtlesim.msg.Pose, callbackTurtlePose, turtleName)
        return suscriptor

    def _initDistanceListener(self):
        turtleTopic = '/' + 'path_requester/' + self.turtleName + '/distance'
        suscriptor = rospy.Subscriber(turtleTopic, std_msgs.msg.Float32, callbackTurtleDistance, turtleName)
        return suscriptor

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

def callbackTurtleDistance(data, turtleName):
    printScalar(data.data, 'Distance')

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
class Controller:
    def __init__(self, publishers):
        self.publishers = publishers

    def _move(self, linearSpeed, angularSpeed, time, refreshTime = 0.5):
        while (time > refreshTime):
            self.publishers.publishSpeed(linearSpeed, angularSpeed)
            rospy.sleep(refreshTime)
            time = time - refreshTime

        self.publishers.publishSpeed(linearSpeed, angularSpeed)
        rospy.sleep(time)
        self.publishers.publishSpeed(0.0, 0.0)

    def _getMovementTimes(self, metrics, angularSpeed, linearSpeed):
        positionDifference = metrics.getPositionDifference()
        # printVector(positionDifference, 'Position difference')

        endAngle = metrics.getEndAngle()
        # printScalar(endAngle, 'End angle')

        rotationTime = metrics.getRotationTime(endAngle, angularSpeed)
        # printScalar(rotationTime, 'Rotation time')

        linearTime = metrics.getTraslationTime(linearSpeed)
        # printScalar(linearTime, 'Traslation time')

        return Vector3(rotationTime, linearTime, 0.0)

    def moveInClosedLoop(self, newPosition, distanceError, angularSpeed, linearSpeed, refreshTime = 0.5):
        #TODO ! Possible data contention
        currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
        currentAngle = currentPose.theta
        #Create the metrics object to encapsulate all the math operations
        metrics = Metrics(currentPosition, currentAngle, newPosition)
        #Get the current error
        currentError = metrics.getDistance()
        #Publish distance
        publishers.publishDistance(currentError)

        while (currentError > distanceError):
            #We get the times for traslation and for rotation
            timeVector = self._getMovementTimes(metrics, angularSpeed, linearSpeed)
            #We make the movements
            self._move(0.0, angularSpeed, timeVector.x, refreshTime)
            self._move(linearSpeed, 0.0, timeVector.y, refreshTime)

            #TODO ! Possible data contention
            currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
            currentAngle = currentPose.theta
            #Create the metrics object to encapsulate all the math operations
            metrics = Metrics(currentPosition, currentAngle, newPosition)
            #Get the current error
            currentError = metrics.getDistance()
            #Publish distance
            publishers.publishDistance(currentError)

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


    def __init__(self, name, publishers, configs):
        self._action_name = name
        #Set publishers and configs
        self._publishers = publishers
        self._configs = configs
        #Create controller
        self._controller = Controller(publishers)
        self._as = actionlib.SimpleActionServer(self._action_name, turtle_controller.msg.PathAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True

        # Set the current distance Accomplished
        self._feedback.currentDistance = 0.0
        
        # publish info to the console for the user
        rospy.loginfo('%s: Controlling turtle to move from here to  %.2f' % (self._action_name, goal.distanceAccomplished))
        
        # start executing the action
        #Move till the turtle gets the position
        #self_controller.moveInClosedLoop(newPosition, 
        #                self._configs.distanceError,
        #                self._configs.angularSpeed,
        #                self._configs.linearSpeed,
        #                self._configs.refreshTime)          

        for i in xrange(1, 10):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            self._feedback.currentDistance = self._feedback.currentDistance + 0.1
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
      
        if success:
            self._result.distanceAccomplished = self._feedback.currentDistance
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
