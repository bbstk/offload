#! /usr/bin/env python

from threading import Thread
from time import time
import subprocess

import rospy
import actionlib

import offload.msg

import socket

class RoutePlanServer(object):
    # create messages that are used to publish feedback/result
    # _feedback = offload.msg.FibonacciFeedback()
    # _result = offload.msg.FibonacciResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(self._action_name, offload.msg.RoutePlanAction, goal_cb=self.goal_cb, auto_start = False)
        self._as.start()
      
    def goal_cb(self, goalHandle):
        goalHandle.set_accepted()
        rospy.loginfo("Accepted new goal")
        def updateStatus(orderNumber):
            feedback = offload.msg.RoutePlanActionFeedback()
            #rospy.loginfo("Attributes: %s", vars(feedback))
            # feedback.currentOrder = orderNumber
            # goalHandle.publish_feedback(feedback)
        
        def completed(finalResult):
            status = goalHandle.get_goal_status().status
            #updateStatus(sequence)

            result = offload.msg.RoutePlanResult()
            result.result = finalResult
            
            if status == actionlib.GoalStatus.ACTIVE:
                goalHandle.set_succeeded(result)
            elif status == actionlib.GoalStatus.PREEMPTED:
                goalHandle.set_preempted(result)
            else:
                goalHandle.set_canceled(result)
        
        def findShortestRoute(goalHandle, statusCB, doneCB):
            start = time()
            goal = goalHandle.get_goal()
            rospy.loginfo('%s: Calculating shortest path from (%i,%i) to (%i,%i)' % (self._action_name, goal.cX, goal.cY, goal.tX, goal.tY))

            subprocess.call("mzn2fzn /home/ubuntu/route-finder-cp/route-finder.mzn /home/ubuntu/route-finder-cp/data.dzn -D\"steps=%d;cX=%d;cY=%d;tX=%d;tY=%d\" -o %f.fzn"%(goal.steps, goal.cX, goal.cY, goal.tX, goal.tY, start), shell=True)
            result = subprocess.check_output("fzn-gecode -p 4 %f.fzn"%(start),shell=True)
            
            end = time()
            rospy.loginfo('%s: Calculating shortest path from (%i,%i) to (%i,%i) in %d seconds' % (self._action_name, goal.cX, goal.cY, goal.tX, goal.tY, end-start))
            doneCB(result)

        Thread(target=findShortestRoute, args=(goalHandle, updateStatus, completed)).start()  
    
if __name__ == '__main__':
    #rospy.init_node('fibonacci')
    rospy.init_node('route_planner_' + socket.gethostname())
    server = RoutePlanServer(rospy.get_name())
    rospy.spin()
