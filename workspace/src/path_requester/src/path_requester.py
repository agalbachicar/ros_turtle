#!/usr/bin/env python

#This program suscribes to the turtle topic of its position
import rospy
import turtlesim.msg
from std_msgs.msg import String


def callbackTurtlePose(data, turtleName):
    rospy.loginfo(rospy.get_caller_id() + " %s : [x: %s; y: %s; theta: %s]", 
        turtleName, data.x, data.y, data.theta)
    
def initPositionListener(turtleName):
    turtleTopic = '/' + turtleName + '/pose'
    suscriptor = rospy.Subscriber(turtleTopic, turtlesim.msg.Pose, callbackTurtlePose, turtleName)
    return suscriptor

def initSpeedPublisher(turtleName):
    rospy.loginfo(rospy.get_caller_id() + " TODO - initSpeedPublisher")
    #turtleTopic = '/' + turtleName + '/cmd_vel'
    #publisher = rospy.Publisher(turtleTopic, String, queue_size=10)

if __name__ == '__main__':
    nodeName = 'ros_turtle_controller'
    turtleName = 'turtle1'
    #Initialize the node
    rospy.init_node(nodeName, anonymous=True)
    #Initialize the turtle position listener
    initPositionListener(turtleName)
    #Initialize the turtle speed publisher
    initSpeedPublisher(turtleName)

    rospy.spin()
  