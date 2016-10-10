#!/usr/bin/env python

#This program suscribes to the turtle topic of its position
import rospy
import turtlesim.msg
from std_msgs.msg import String


def callback(data, turtleName):
    rospy.loginfo(rospy.get_caller_id() + " %s : [x: %s; y: %s; theta: %s]", 
        turtleName, data.x, data.y, data.theta)
    
def listener(turtleName):
    listenerName = 'positionListener'
    rospy.init_node(listenerName, anonymous=True)
    turtleTopic = '/' + turtleName + '/pose'

    rospy.Subscriber(turtleTopic, turtlesim.msg.Pose, callback, turtleName)

    rospy.spin()

if __name__ == '__main__':
    listener('turtle1')
