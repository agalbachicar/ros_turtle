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
    
class PathServerAction(object):
    # create messages that are used to publish feedback/result
    _feedback = turtle_controller.msg.PathFeedback()
    _result   = turtle_controller.msg.PathResult()

    def __init__(self, name):
        self._action_name = name
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
  rospy.loginfo('Path Server initiated.')
  rospy.loginfo('Listening for incoming goals.')
  PathServerAction(rospy.get_name())
  rospy.spin()