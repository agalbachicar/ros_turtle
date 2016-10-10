#!/usr/bin/env python

#This program suscribes to the turtle topic of its position
import rospy
import turtlesim.msg
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String
import math

#Current position of the turtle
currentPose = turtlesim.msg.Pose()

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

def logCurrentPose(turtleName):
    rospy.loginfo(rospy.get_caller_id() + " %s : [x: %.2f; y: %.2f; theta: %.2f] [linear: %.2f, angular: %.2f]", 
        turtleName, currentPose.x, currentPose.y, currentPose.theta, currentPose.linear_velocity, currentPose.angular_velocity)    

def printCurrentXYAngle(turtleName):
    rospy.loginfo("--> Current %s position: [%.2f ; %.2f ]-[%.4f]", turtleName, currentPose.x, currentPose.y, currentPose.theta)

def printVector(vector, str = ''):
    rospy.loginfo("--> %s: [%.2f ; %.2f ; %.2f]", str, vector.x, vector.y, vector.z)

def printScalar(time, str = ''):
    rospy.loginfo("--> %s: [%.4f]", str, time)

def initPositionListener(turtleName):
    turtleTopic = '/' + turtleName + '/pose'
    suscriptor = rospy.Subscriber(turtleTopic, turtlesim.msg.Pose, callbackTurtlePose, turtleName)
    return suscriptor

def initSpeedPublisher(turtleName):
    turtleTopic = '/' + turtleName + '/cmd_vel'
    publisher = rospy.Publisher(turtleTopic, Twist, queue_size=10)
    return publisher

# TODO! Argument check!
def getNewPositionFromCLI():
    x = raw_input("--> Please insert the new X coordinate: ")
    y = raw_input("--> Please insert the new Y coordinate: ")
    return Vector3(float(x), float(y), 0.0)

# TODO! Possible division by zero
def calculateTraslationTime(currentPosition, newPosition, speed):
    delta = getPositionDifference(currentPosition, newPosition)
    return math.sqrt( (math.pow(delta.x, 2.0) + math.pow(delta.y, 2.0)) / speed)

# TODO! Possible division by zero
def calculateRotationTime(currentAngle, newAngle, angularSpeed):
    delta = getAngleDifference(currentAngle, newAngle)
    return ( delta / angularSpeed )

# TODO! Possible division by zero
def calculateEndAngle(currentPosition, newPosition):
    delta = getPositionDifference(currentPosition, newPosition)
    return math.atan2(delta.y, delta.x)

def getPositionDifference(currentPosition, newPosition):
    dx = newPosition.x - currentPosition.x
    dy = newPosition.y - currentPosition.y
    dz = newPosition.z - currentPosition.z
    return Vector3(dx, dy, dz)

def getAngleDifference(currentAngle, newAngle):
    cAngle = getAngleIn2PIModulus(currentAngle)
    nAngle = getAngleIn2PIModulus(newAngle)
    difAngle = getAngleIn2PIModulus(nAngle - cAngle)
    return difAngle

def getAngleIn2PIModulus(angle):
    if angle < 0.0:
        return math.pi + math.pi + angle
    else:
        return angle

def getCleanVector():
    return Vector3(0.0, 0.0, 0.0)

def move(publisher, traslationVector, rotationVector, time, refreshTime = 0.5):
    while (time > refreshTime):
        publisher.publish(Twist(traslationVector, rotationVector))
        rospy.sleep(refreshTime)
        time = time - refreshTime

    publisher.publish(Twist(traslationVector, rotationVector))
    rospy.sleep(time)
    publisher.publish(Twist(getCleanVector(), getCleanVector()))
    return

def getDistance(currentPosition, newPosition):
    diffPosition = getPositionDifference(currentPosition, newPosition)
    return math.sqrt(diffPosition.x * diffPosition.x + diffPosition.y * diffPosition.y)

def getMovementTimes(currentPosition, currentAngle, newPosition, angularSpeed, linearSpeed):
    #Get the position difference
    positionDifference = getPositionDifference(currentPosition, newPosition)
    printVector(positionDifference, 'Position difference')
    #Get the end angle
    endAngle = calculateEndAngle(currentPosition, newPosition)
    printScalar(endAngle, 'End angle')
    #Get the rotation time
    rotationTime = calculateRotationTime(currentAngle, endAngle, angularSpeed)
    printScalar(rotationTime, 'Rotation time')
    #Calulo el tiempo de traslacion
    linearTime = calculateTraslationTime(currentPosition, newPosition, linearSpeed)
    printScalar(linearTime, 'Traslation time')    
    return Vector3(rotationTime, linearTime, 0.0);

def moveInClosedLoop(newPosition, distanceError, angularSpeed, linearSpeed, publisher, refreshTime = 0.5):
    #TODO ! Possible data contention
    currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
    currentAngle = currentPose.theta
    #Get the current error
    currentError = getDistance(currentPosition, newPosition)

    while (currentError > distanceError):
        #We get the times for traslation and for rotation
        timeVector = getMovementTimes(currentPosition, currentAngle, newPosition, angularSpeed, linearSpeed)
        #We make the movements
        move(publisher, getCleanVector(), Vector3(0.0, 0.0, angularSpeed), timeVector.x, refreshTime)
        move(publisher, Vector3(linearSpeed, 0.0, 0.0), getCleanVector(), timeVector.y, refreshTime)

        #TODO ! Possible data contention
        currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
        currentAngle = currentPose.theta
        #Get the current error
        currentError = getDistance(currentPosition, newPosition)

    return

if __name__ == '__main__':
    nodeName = 'ros_turtle_controller'
    turtleName = 'turtle1'

    rateFrequency = 1 #in hertz

    linearSpeed = 2.0
    angularSpeed = 2.0
    distanceError = 0.1
    refreshTime = 0.01

    #Initialize the node
    rospy.init_node(nodeName, anonymous=True)
    #Initialize the turtle position listener
    initPositionListener(turtleName)
    #Initialize the turtle speed publisher
    speedPublisher = initSpeedPublisher(turtleName)
    #Get the rate of the main thread
    rate = rospy.Rate(rateFrequency)

    while not rospy.is_shutdown():
        #Get the new position
        printCurrentXYAngle(turtleName)
        newPosition = getNewPositionFromCLI()
        #Move till the turtle gets the postion
        moveInClosedLoop(newPosition, distanceError, angularSpeed, linearSpeed, speedPublisher, refreshTime)

        # #TODO ! Possible data contention
        # currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
        # currentAngle = currentPose.theta
        # printVector(currentPosition, 'Current position')
        # printScalar(currentAngle, 'Current angle')

        # #Get the position difference
        # positionDifference = getPositionDifference(currentPosition, newPosition)
        # printVector(positionDifference, 'Position difference')
        # #Get the end angle
        # endAngle = calculateEndAngle(currentPosition, newPosition)
        # printScalar(endAngle, 'End angle')
        # #Get the rotation time
        # rotationTime = calculateRotationTime(currentAngle, endAngle, angularSpeed)
        # printScalar(rotationTime, 'Rotation time')
        # #Calulo el tiempo de traslacion
        # linearTime = calculateTraslationTime(currentPosition, newPosition, linearSpeed)
        # printScalar(linearTime, 'Traslation time')

        # #We send the rotation, linear traslation and stop command 
        # move(speedPublisher, getCleanVector(),  Vector3(0.0, 0.0, angularSpeed), rotationTime)
        # move(speedPublisher, Vector3(linearSpeed, 0.0, 0.0),  getCleanVector(), linearTime)
  