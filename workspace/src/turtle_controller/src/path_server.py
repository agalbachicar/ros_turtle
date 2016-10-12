#! /usr/bin/env python

import roslib; #roslib.load_manifest('turtle_controller')
import rospy

import actionlib

import turtle_controller.msg

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