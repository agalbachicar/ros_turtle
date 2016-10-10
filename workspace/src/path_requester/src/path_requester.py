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

def printCurrentXY(turtleName):
    rospy.loginfo("--> Current %s position: [%.2f ; %.2f ]", turtleName, currentPose.x, currentPose.y)

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
    return ( newAngle - currentAngle )

if __name__ == '__main__':
    nodeName = 'ros_turtle_controller'
    turtleName = 'turtle1'

    rateFrequency = 1 #in hertz

    linearSpeed = 1.0
    angularSpeed = 1.0

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
        printCurrentXY(turtleName)
        newPosition = getNewPositionFromCLI()

        #TODO ! Possible data contention
        currentPosition = Vector3(currentPose.x, currentPose.y, 0.0)
        currentAngle = currentPose.theta

        #Get the end angle
        endAngle = calculateEndAngle(currentPosition, newPosition)
        #Get the rotation time
        rotationTime = calculateRotationTime(currentAngle, endAngle, angularSpeed)
        #Calulo el tiempo de traslacion
        linearTime = calculateTraslationTime(currentPosition, newPosition, linearSpeed)

        #We send the rotation, linear traslation and stop command 
        speedPublisher.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, angularSpeed)))
        rospy.sleep(rotationTime)
        speedPublisher.publish(Twist(Vector3(linearSpeed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
        rospy.sleep(linearTime)
        speedPublisher.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

        #rate.sleep()
    
    #rospy.spin()
  