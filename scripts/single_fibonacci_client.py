#! /usr/bin/env python
from __future__ import print_function
import rospy
import sys

import actionlib
from actionlib_msgs.msg import *

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import offload.msg

def fibonacci_client(n, repetitions):
    # Creates the ActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.ActionClient('fibonacci', offload.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = offload.msg.FibonacciGoal(order=n)

    # Create list to store the ClientActionHandlers
    handlers = []

    # Send the specified number of requests
    for i in range(0, repetitions):
        handlers.append(client.send_goal(goal))
        # wait for the goal to be received
        while handlers[i].get_goal_status() == GoalStatus.PENDING:
            continue

    rospy.loginfo("handlers len: %d", len(handlers))

    # Wait until each handler return successfully
    for i in range(0, repetitions):
        while handlers[i].get_goal_status() != GoalStatus.SUCCEEDED:
            rospy.loginfo("handlers[%d].status == %s", i, handlers[i].get_goal_status())
            rospy.sleep(0.5)
            continue

    # Prints out the result of executing the action
    return handlers[0].get_result()  # A FibonacciResult

def usage():
    return "%s [n] [repetitions]"%sys.argv[0]

if __name__ == '__main__':
    if len(sys.argv) == 3:
        n = int(sys.argv[1])
        repetitions = int(sys.argv[2])
    else:
        print (usage())
        sys.exit(1)
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('single_fibonacci_client_py')
        result = fibonacci_client(n, repetitions)
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
