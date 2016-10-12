#!/usr/bin/env python

#This program suscribes to the turtle topic of its position
import rospy
import turtlesim.msg
from geometry_msgs.msg import Twist, Vector3
import std_msgs.msg
import math

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

# TODO! Argument check!
def getNewPositionFromCLI():
    x = raw_input("--> Please insert the new X coordinate: ")
    y = raw_input("--> Please insert the new Y coordinate: ")
    return Vector3(float(x), float(y), 0.0)

if __name__ == '__main__':
    nodeName = 'ros_turtle_controller'
    turtleName = 'turtle1'

    #Load the configurations
    configs = Configs()

    #Initialize the node
    rospy.init_node(nodeName, anonymous=True)
    
    #Initialize the turtle position listener
    listener = Listeners(turtleName)
    #Initialize the publishers
    publishers = Publishers(turtleName)

    #Initialize the controller
    controller = Controller(publishers)
    #Get the rate of the main thread
    rate = rospy.Rate(configs.refreshTime)

    while not rospy.is_shutdown():
        #Get the new position
        printCurrentXYAngle(turtleName)
        newPosition = getNewPositionFromCLI()
        #Move till the turtle gets the position
        controller.moveInClosedLoop(newPosition, 
                        configs.distanceError,
                        configs.angularSpeed,
                        configs.linearSpeed,
                        configs.refreshTime)
  