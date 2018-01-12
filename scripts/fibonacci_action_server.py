#! /usr/bin/env python

from threading import Thread

import rospy
import actionlib

import offload.msg

import socket

class FibonacciAction(object):
    # create messages that are used to publish feedback/result
    # _feedback = offload.msg.FibonacciFeedback()
    # _result = offload.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(self._action_name, offload.msg.FibonacciAction, goal_cb=self.goal_cb, auto_start = False)
        self._as.start()
      
    def goal_cb(self, goalHandle):
        goalHandle.set_accepted()
        rospy.loginfo("Accepted new goal")
        def updateStatus(orderNumber):
            feedback = offload.msg.FibonacciActionFeedback()
            #rospy.loginfo("Attributes: %s", vars(feedback))
            feedback.currentOrder = orderNumber
            goalHandle.publish_feedback(feedback)
        
        def completed(finalResult):
            status = goalHandle.get_goal_status().status
            #updateStatus(sequence)

            result = offload.msg.FibonacciResult()
            result.result = finalResult

            if status == actionlib.GoalStatus.ACTIVE:
                goalHandle.set_succeeded(result)
            elif status == actionlib.GoalStatus.PREEMPTED:
                goalHandle.set_preempted(result)
            else:
                goalHandle.set_canceled(result)
        
        def calculateFibonacci(goalHandle, statusCB, doneCB):
            goal = goalHandle.get_goal()
            rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i' % (self._action_name, goal.order))
            # Return the result from the recursive fib() function
            doneCB(fib(goal.order))

        def fib(n):
           if n == 1:
              return 1
           elif n == 0:
              return 0
           else:
              return fib(n-1) + fib(n-2)

        Thread(target=calculateFibonacci, args=(goalHandle, updateStatus, completed)).start()  
    
if __name__ == '__main__':
    #rospy.init_node('fibonacci')
    rospy.init_node('fib_server_' + socket.gethostname())
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
