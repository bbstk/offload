#! /usr/bin/env python

from threading import Thread

import rospy
import actionlib

import offload.msg

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
        def updateStatus(sequence):
            feedback = offload.msg.FibonacciActionFeedback()
            #rospy.loginfo("Attributes: %s", vars(feedback))
            feedback.feedback = sequence
            goalHandle.publish_feedback(feedback)
        
        def completed(sequence):
            status = goalHandle.get_goal_status().status
            #updateStatus(sequence)

            result = offload.msg.FibonacciResult()
            result.sequence = sequence

            if status == actionlib.GoalStatus.ACTIVE:
                goalHandle.set_succeeded(result)
            elif status == actionlib.GoalStatus.PREEMPTED:
                goalHandle.set_preempted(result)
            else:
                goalHandle.set_canceled(result)
        
        def calculateFibonacci(goalHandle, statusCB, doneCB):
            goal = goalHandle.get_goal()
            sequence = []
            sequence.append(0)
            sequence.append(1)
            rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, sequence[0], sequence[1]))
            
            r = rospy.Rate(2)

            for i in range(1, goal.order):
                # check that preempt has not been requested by the client
                # status = goalHandle.get_goal_status().status
                # if status == actionlib.GoalStatus.PREEMPTED:
                #     do something
                
                sequence.append(sequence[i] + sequence[i-1])
                
                # publish the feedback
                
                # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
                r.sleep()

            doneCB(sequence)

        Thread(target=calculateFibonacci, args=(goalHandle, updateStatus, completed)).start()
  





        # # helper variables
        # r = rospy.Rate(1)
        # success = True
        
        # # append the seeds for the fibonacci sequence
        # self._feedback.sequence = []
        # self._feedback.sequence.append(0)
        # self._feedback.sequence.append(1)
        
        # # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        
        # # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        # if success:
        #     self._result.sequence = self._feedback.sequence
        #     rospy.loginfo('%s: Succeeded' % self._action_name)
        #     self._as.set_succeeded(self._result)
    
if __name__ == '__main__':
    rospy.init_node('fibonacci')
    server = FibonacciAction(rospy.get_name())
    rospy.spin()
