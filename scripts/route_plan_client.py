#! /usr/bin/env python
from __future__ import print_function
from time import time
import rospy
import sys
import socket
import threading

import actionlib
from actionlib_msgs.msg import *
import offload.msg
from route_plan_aac import RoutePlanAutonomousActionClient
from route_planning_autonomous_program import RoutePlanningAutonomousProgram

node_addresses = ['pi1', 'pi2', 'pi3', 'pi4', 'pi5']

def single_server(cX, cY, tX, tY, steps):
    client = actionlib.ActionClient('route_planner_' + socket.gethostname(), offload.msg.RoutePlanAction)
    client.wait_for_server()
    goal = offload.msg.RoutePlanGoal(cX=cX, cY=cY, tX=tX, tY=tY, steps=steps)
    handler = client.send_goal(goal)
    while handler.get_goal_status() != GoalStatus.SUCCEEDED:
        rospy.sleep(0.2)

    return handler.get_result()

def specific_server(cX, cY, tX, tY, steps, server):
    client = actionlib.ActionClient('route_planner_' + server, offload.msg.RoutePlanAction)
    client.wait_for_server()
    goal = offload.msg.RoutePlanGoal(cX=cX, cY=cY, tX=tX, tY=tY, steps=steps)
    handler = client.send_goal(goal)
    while handler.get_goal_status() != GoalStatus.SUCCEEDED:
        rospy.sleep(0.2)

    return handler.get_result()

def round_robin_server(cX, cY, tX, tY, steps, repetitions):
    # Create the clients
    clients = []
    for address in node_addresses:
        clients.append(actionlib.ActionClient('route_planner_' + address, offload.msg.RoutePlanAction))

    # Wait for the servers
    for client in clients:
        client.wait_for_server()

    # Create the goal
    goal = offload.msg.RoutePlanGoal(cX=cX, cY=cY, tX=tX, tY=tY, steps=steps)

    # Create list to store the ClientActionHandlers
    handlers = []

    sendStart = time()
    # Send the requests in round robin fashion
    for i in range(0, repetitions):
        handlers.append(clients[i%len(clients)].send_goal(goal))
        # wait for the goal from the same client to be received
        while i >= len(clients) - 1 and handlers[i + 1 -len(clients)].get_goal_status() == GoalStatus.PENDING:
            rospy.sleep(0.1)
            continue
    sendEnd = time()
    rospy.loginfo("Sent all goals in %d seconds", sendEnd - sendStart)


    # Wait until each handler return successfully
    for i in range(0, repetitions):
        while handlers[i].get_goal_status() != GoalStatus.SUCCEEDED:
            #rospy.loginfo("handlers[%d].status == %s", i, handlers[i].get_goal_status())
            rospy.sleep(0.2)

    # Prints out the result of executing the action
    return handlers[0].get_result()  # A FibonacciResult

def amp_server(cX, cY, tX, tY, steps):
    aac = RoutePlanAutonomousActionClient(offload.msg.RoutePlanAction)
    goal = offload.msg.RoutePlanGoal(cX=cX, cY=cY, tX=tX, tY=tY, steps=steps)

    sendStart = time()
    handler = aac.send_goal(goal)
    sendEnd = time()
    rospy.loginfo("Sent all goals in %d seconds", sendEnd - sendStart)

    while handler.get_goal_status() != GoalStatus.SUCCEEDED:
        rospy.sleep(0.2)

    return handler.get_result() 


ampft_done = threading.Event()
ampft_result = ""

def ampft_server(cX, cY, tX, tY, steps):
    aac = RoutePlanningAutonomousProgram(offload.msg.RoutePlanAction)
    goal = offload.msg.RoutePlanGoal(cX=cX, cY=cY, tX=tX, tY=tY, steps=steps)

    sendStart = time()
    handler = aac.send_goal(goal, ampft_callback)
    sendEnd = time()
    rospy.loginfo("Sent all goals in %d seconds", sendEnd - sendStart)

    global ampft_done
    ampft_done.wait()

    return ampft_result

def ampft_callback(status, result):
    global ampft_done
    global ampft_result
    ampft_result = result
    ampft_done.set()

ampft_burst_repetitions = 0
ampft_burst_done = threading.Event()
ampft_burst_counter = 0
ampft_burst_lock = threading.Lock()

def ampft_burst_server(cX, cY, tX, tY, steps, repetitions):
    clients = []
    for i in range(repetitions):
        clients.append(RoutePlanningAutonomousProgram(offload.msg.RoutePlanAction))
    goal = offload.msg.RoutePlanGoal(cX=cX, cY=cY, tX=tX, tY=tY, steps=steps)

    global ampft_burst_repetitions
    ampft_burst_repetitions = repetitions

    for i in range(repetitions):
        clients[i].send_goal(goal, ampft_burst_callback)

    global ampft_burst_done
    ampft_burst_done.wait()
    return "done"

def ampft_burst_callback(status, result):
    global ampft_burst_done
    global ampft_burst_counter
    global ampft_burst_lock
    with ampft_burst_lock:
        ampft_burst_counter += 1
        if ampft_burst_counter == ampft_burst_repetitions:
            ampft_burst_done.set()

def usage():
    return "%s [cX] [xY] [tX] [tY] [steps] [single|amp|ampft|ampft_burst {repetitions}|specific {server name}|rr {repetitions}]"%sys.argv[0]

if __name__ == '__main__':
    if len(sys.argv) == 7 and sys.argv[6] in ['single', 'amp', 'ampft']:
        cX = int(sys.argv[1])
        cY = int(sys.argv[2])
        tX = int(sys.argv[3])
        tY = int(sys.argv[4])
        steps = int(sys.argv[5])
        mode = sys.argv[6]
    elif len(sys.argv) == 8 and sys.argv[6] in ['specific', 'ampft_burst', 'rr']:
        cX = int(sys.argv[1])
        cY = int(sys.argv[2])
        tX = int(sys.argv[3])
        tY = int(sys.argv[4])
        steps = int(sys.argv[5])
        mode = sys.argv[6]
        arg7 = sys.argv[7]
    else:
        print (usage())
        sys.exit(1)
    try:
        rospy.init_node('route_plan_client', anonymous=True)
        result = []
        if mode == 'single':
            result = single_server(cX, cY, tX, tY, steps)
        elif mode == 'amp':
            result = amp_server(cX, cY, tX, tY, steps)
        elif mode == 'ampft':
            result = ampft_server(cX, cY, tX, tY, steps)
        elif mode == 'ampft_burst':
            result = ampft_burst_server(cX, cY, tX, tY, steps, int(arg7))
        elif mode == 'specific':
            result = specific_server(cX, cY, tX, tY, steps, arg7)
        elif mode == 'rr':
            result = round_robin_server(cX, cY, tX, tY, steps, int(arg7))
        else:
            print (usage())
            sys.exit(1)
#        print("Result: ", result.result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
