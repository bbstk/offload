#! /usr/bin/env python
from __future__ import print_function
import rospy
import sys
import socket

import actionlib
from actionlib_msgs.msg import *
import offload.msg

node_addresses = ['pi1', 'pi2', 'virtualpi1']

def fibonacci_client_single_server(n, repetitions):
    # Creates the ActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.ActionClient('fib_server_' + socket.gethostname(), offload.msg.FibonacciAction)

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
        status = handlers[i].get_goal_status()
        rospy.loginfo("handlers[%d].status == %s", i, status)
        while status != GoalStatus.SUCCEEDED:
            rospy.sleep(0.2)
            status = handlers[i].get_goal_status()
            rospy.loginfo("handlers[%d].status == %s", i, status)

    # Prints out the result of executing the action
    return handlers[0].get_result()  # A FibonacciResult

def fibonacci_client_round_robin_server(n, repetitions):
    # Create the clients
    clients = []
    for address in node_addresses:
        clients.append(actionlib.ActionClient(address, offload.msg.FibonacciAction))

    # Wait for the servers
    for client in clients:
        client.wait_for_server()

    # Create the goal
    goal = offload.msg.FibonacciGoal(order=n)

    # Create list to store the ClientActionHandlers
    handlers = []

    # Send the requests in round robin fashion
    for i in range(0, repetitions):
        handlers.append(clients[i%len(clients)].send_goal(goal))
        # wait for the goal to be received
        while handlers[i].get_goal_status() == GoalStatus.PENDING:
            continue

    # Wait until each handler return successfully
    for i in range(0, repetitions):
        while handlers[i].get_goal_status() != GoalStatus.SUCCEEDED:
            rospy.loginfo("handlers[%d].status == %s", i, handlers[i].get_goal_status())
            rospy.sleep(0.2)
            continue

    # Prints out the result of executing the action
    return handlers[0].get_result()  # A FibonacciResult

def usage():
    return "%s [n] [repetitions] [single|rr|amp]"%sys.argv[0]

if __name__ == '__main__':
    if len(sys.argv) == 4 and sys.argv[3] in ['single', 'rr', 'amp']:
        n = int(sys.argv[1])
        repetitions = int(sys.argv[2])
        mode = sys.argv[3]
    else:
        print (usage())
        sys.exit(1)
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_action_client_py')
        result = []
        if mode == 'single':
            result = fibonacci_client_single_server(n, repetitions)
        elif mode == 'rr':
            result = fibonacci_client_round_robin_server(n, repetitions)
        elif mode == 'amp':
            print ('Not implemented')
            sys.exit(1)
        else:
            print (usage())
            sys.exit(1)
        print("Result: ", result.result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
