#!/usr/bin/env python

#This program suscribes to the turtle topic of its position
import rospy
import turtlesim.msg
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

#Current position of the turtle
turtlePose = turtlesim.msg.Pose()

def callbackTurtlePose(data, turtleName):
    if (data.x != turtlePose.x or 
        data.y != turtlePose.y or 
        data.theta != turtlePose.theta or 
        data.linear_velocity != data.linear_velocity or 
        data.angular_velocity != data.angular_velocity):
        #Copy current pose
        turtlePose.x = data.x
        turtlePose.y = data.y
        turtlePose.theta = data.theta
        turtlePose.linear_velocity = data.linear_velocity
        turtlePose.angular_velocity = data.angular_velocity
        #Log new values
        rospy.loginfo(rospy.get_caller_id() + " %s : [x: %s; y: %s; theta: %s]", 
            turtleName, data.x, data.y, data.theta)
    
def initPositionListener(turtleName):
    turtleTopic = '/' + turtleName + '/pose'
    suscriptor = rospy.Subscriber(turtleTopic, turtlesim.msg.Pose, callbackTurtlePose, turtleName)
    return suscriptor

def initSpeedPublisher(turtleName):
    turtleTopic = '/' + turtleName + '/cmd_vel'
    publisher = rospy.Publisher(turtleTopic, Twist, queue_size=10)
    return publisher

if __name__ == '__main__':
    nodeName = 'ros_turtle_controller'
    turtleName = 'turtle1'
    rateFrequency = 1 #in hertz

    #Initialize the node
    rospy.init_node(nodeName, anonymous=True)
    #Initialize the turtle position listener
    initPositionListener(turtleName)
    #Initialize the turtle speed publisher
    speedPublisher = initSpeedPublisher(turtleName)
    #Get the rate of the main thread
    rate = rospy.Rate(rateFrequency)

    while not rospy.is_shutdown():
        speedPublisher.publish(Twist(Vector3(-1.0, 0, 0), Vector3(0, 0, 0)))
        rate.sleep()
    
    rospy.spin()
  