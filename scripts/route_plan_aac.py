#! /usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
from offload.srv import *
import offload.msg
import socket
import math

class RoutePlanAutonomousActionClient:
    def __init__(self, ActionSpec):
        self.action_spec = ActionSpec

    ## @brief Sends a goal to the an ActionServer decided using a cost model
    ##
    ## @param goal An instance of the *Goal message.
    ##
    ## @param transition_cb Callback that gets called on every client
    ## state transition for the sent goal.  It should take in a
    ## ClientGoalHandle as an argument.
    ##
    ## @param feedback_cb Callback that gets called every time
    ## feedback is received for the sent goal.  It takes two
    ## parameters: a ClientGoalHandle and an instance of the *Feedback
    ## message.
    ##
    ## @return ClientGoalHandle for the sent goal.
    def send_goal(self, goal, transition_cb = None, feedback_cb = None):
        # get best action server to use
        server_to_use = best_server(goal)
        rospy.loginfo("Best server name: " + server_to_use)
	#TODO: store clients in a map? so that you dont create a new one every time. (Client pool?)
        client = actionlib.ActionClient(server_to_use, self.action_spec)
        #TODO: wait a certain amount, if timeout -> send to another server
        client.wait_for_server()
        return client.send_goal(goal, transition_cb, feedback_cb)


# fibPerformanceMap = {
#   32:{
#       0: 14.1,
#       25: 14.2,
#       50: 14.3,
#       75: 15.6,
#       100: 16.2,
#   }
# }


# return the best server to use for a specific goal
def best_server(goal):
    blackboard = rospy.ServiceProxy('stats_reporter_' + socket.gethostname(), Blackboard)
    load_info = blackboard("")

#    rospy.loginfo("Load info %r", load_info)

    current_node = socket.gethostname()

    current_best = {}
    current_best["name"] = socket.gethostname()
    current_best["busyCores"] = 4

    # current_best = offload.msg.SystemStats()
    # current_best.name = socket.gethostname()
    # current_best.cpuUsage = 900.0
    # current_best.availableMemory = 0
 #   rospy.loginfo(goal)
    for node in load_info.result:
        # timeComp = fibPerformanceMap[goal.order][int(math.ceil(node.cpuUsage / 25.0) * 25)]
        # rospy.loginfo(node.name)
        # rospy.loginfo(timeComp)
        # timeComm = 0;
        # if node.name != current_node:
        #     timeComm = 0.006;
        # timeTotal = timeComp + timeComm
        busyCores = 100/int(math.ceil(node.cpuUsage / 25.0) * 25)
        if timeTotal < current_best["timeTotal"]:
            current_best["name"] = node.name
            current_best["busyCores"] = busyCores

    return "route_planner_" + current_best["name"]
