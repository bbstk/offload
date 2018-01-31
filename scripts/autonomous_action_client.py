#! /usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import *
from offload.srv import *
import socket

class AutonomousActionClient:
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
        #TODO: store clients in a map? so that you dont create a new one every time. (Client pool?)
        client = actionlib.ActionClient(server_to_use, self.action_spec)
        #TODO: wait a certain amount, if timeout -> send to another server
        client.wait_for_server()
        return client.send_goal(goal, transition_cb, feedback_cb)


# return the best server to use for a specific goal
def best_server(goal):
    blackboard = rospy.ServiceProxy('stats_reporter_' + socket.gethostname(), Blackboard)
    load_info = blackboard("")
    rospy.loginfo("Load info %r", load_info)
    return "fib_server_virtualpi1"
