#!/usr/bin/env python
import rospy
import offload.msg
from offload.srv import *

import socket

def callback(data):
    start = time()
    result = fib(data.data)
    end = time()
    rospy.loginfo(rospy.get_caller_id() + "| Fib(%d) is %d and it took me %d seconds to calculate", data.data, result, end-start)
    
def fib_solver():
    rospy.init_node('fib_solver', anonymous=True)

    rospy.Subscriber("fib_tasks", UInt32, callback)

    rospy.spin()

class Blackboard(object):
    stats = {}

    def __init__(self, nodes):
        for node in nodes:
          self.Subscribers.append(rospy.Subscriber("stats_" + node, offload.msg.SystemStats, self.record_stats, (node)))
        self.reporter = rospy.Service('stats_reporter_' + socket.gethostname(), Blackboard, self.report)
    
    def record_stats(data, node_name):
      stats[node_name] = data

    def report(node_name):
      return stats[node_name]

if __name__ == '__main__':
    nodes = ['pi1', 'pi2', 'virtualpi1']
    rospy.init_node('blackboard_' + socket.gethostname())
    server = Blackboard(nodes)
    rospy.spin()