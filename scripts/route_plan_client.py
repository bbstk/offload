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
from route_plan_aac_ft import RoutePlanAutonomousActionClientFT

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
    aac = RoutePlanAutonomousActionClientFT(offload.msg.RoutePlanAction)
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


def usage():
    return "%s [cX] [xY] [tX] [tY] [steps] [single|amp|ampft|specific {server name}]"%sys.argv[0]

if __name__ == '__main__':
    if len(sys.argv) == 7 and sys.argv[6] in ['single', 'amp', 'ampft']:
        cX = int(sys.argv[1])
        cY = int(sys.argv[2])
        tX = int(sys.argv[3])
        tY = int(sys.argv[4])
        steps = int(sys.argv[5])
        mode = sys.argv[6]
    elif len(sys.argv) == 8 and sys.argv[6] in ['specific']:
        cX = int(sys.argv[1])
        cY = int(sys.argv[2])
        tX = int(sys.argv[3])
        tY = int(sys.argv[4])
        steps = int(sys.argv[5])
        mode = sys.argv[6]
        server = sys.argv[7]
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
        elif mode == 'specific':
            result = specific_server(cX, cY, tX, tY, steps, server)
        else:
            print (usage())
            sys.exit(1)
        print("Result: ", result.result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
