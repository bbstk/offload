#! /usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
from offload.srv import *
import offload.msg
import socket
import math
import random
import os
from threading import Thread

class RoutePlanAutonomousActionClientFT:
    def __init__(self, ActionSpec):
        self.action_spec = ActionSpec
        self.blackboard = rospy.ServiceProxy('blackboard_ft_' + socket.gethostname(), BlackboardFT)

    ## @brief Sends a goal to the an ActionServer decided using a cost model
    ##
    ## @param goal An instance of the *Goal message.
    ##
    ## @param done_cb Callback that gets called on transitions to Done
    ## The callback should take 2 parameter - status code and the result
    ##
    def send_goal(self, goal, done_cb = None):
        self.is_done = False
        self.goal = goal
        self.done_cb = done_cb
        # get best action server to use
        self.server_to_use = best_server(goal)
        rospy.loginfo("Best server name: " + self.server_to_use)
        self.client = actionlib.SimpleActionClient("route_planner_" + self.server_to_use, self.action_spec)
        #TODO: wait a certain amount, if timeout -> send to another server
        self.client.wait_for_server()
        self.client.send_goal(goal = goal, done_cb = self._handle_transition)
        Thread(target=self._ping_server).start()


    def _handle_transition(self, status, result):
	rospy.loginfo("Calling done cb")
        self.is_done = True
        self.done_cb(status, result)
        #comm_state = gh.get_comm_state()
        #rospy.loginfo(comm_state)
        #if comm_state == CommState.DONE:
         #   if self.done_cb:
          #      self.done_cb(gh.get_goal_status(), gh.get_result())

    def _ping_server():
        while not self.is_done:
            rospy.sleep(2)
            response = os.system("ping -c 1 -w2 " + self.server_to_use + " > /dev/null 2>&1")
            if response != 0:
                self.blackboard(self.server_to_use, True)
                self.send_goal(self.goal, self.done_cb)
                return



# return the best server to use for a specific goal
def best_server(goal):
    load_info = self.blackboard("",False)

#    rospy.loginfo("Load info %r", load_info)

    current_node = socket.gethostname()

    current_best = {}
    current_best["name"] = socket.gethostname()
    #current_best["busyCores"] = 4
    current_best["timeTotal"] = 99999     

    # current_best = offload.msg.SystemStats()
    # current_best.name = socket.gethostname()
    # current_best.cpuUsage = 900.0
    # current_best.availableMemory = 0
 #   rospy.loginfo(goal)
    
    random.shuffle(load_info.result)
    for node in load_info.result:
        # timeComp = fibPerformanceMap[goal.order][int(math.ceil(node.cpuUsage / 25.0) * 25)]
        rospy.loginfo(node.name)
        rospy.loginfo("cpu usage: %f"%(node.cpuUsage))
	# rospy.loginfo(timeComp)
        timeComp = 10 + 0.1 * int(math.ceil(node.cpuUsage / 10.0) * 10)
	timeComm = 0;
        if node.name != current_node:
            timeComm = 0.006;
        timeTotal = timeComp + timeComm
#        busyCores = 100/int(math.ceil(node.cpuUsage / 25.0) * 25)
#	rospy.loginfo("busy cores: %d"%(busyCores))
	rospy.loginfo("time total: %f"%(timeTotal)) 
        if timeTotal < current_best["timeTotal"]:# or (timeTotal == current_best["timeTotal"] and random.randint(0,1) == 1):
#	if busyCores < current_best["busyCores"] or (busyCores == current_best["busyCores"] and random.randint(0,1) == 1):
            current_best["name"] = node.name
            current_best["timeTotal"] = timeTotal

    return current_best["name"]
