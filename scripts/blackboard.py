#!/usr/bin/env python
import rospy
import offload.msg
from offload.srv import *

import socket

class BlackboardCls:
    stats = {}
    subscribers = []
    def __init__(self, nodes):
        for node in nodes:
          self.subscribers.append(rospy.Subscriber("stats_" + node, offload.msg.SystemStats, self.record_stats, (node), queue_size=10))
        self.reporter = rospy.Service('stats_reporter_' + socket.gethostname(), Blackboard, self.report)
    
    def record_stats(self, data, node_name):
      #rospy.loginfo("Storing data for %s", node_name)
      to_store = offload.msg.SystemStats()
      to_store.name = data.name
      to_store.cpuUsage = data.cpuUsage
      to_store.availableMemory = data.availableMemory
      self.stats[node_name] = to_store

    def report(self, request):
      node_name = request.nodeName
      #rospy.loginfo("Data requested for %r", node_name)
      #Default response is an infinitely busy system
      default = offload.msg.SystemStats()
      default.cpuUsage = 100.0
      default.availableMemory = 0
      resp = offload.srv.BlackboardResponse()
      i = 0
      for key, value in self.stats.items():
        resp.result.append(offload.msg.SystemStats())
        resp.result[i].name = value.name
        resp.result[i].cpuUsage = value.cpuUsage
        resp.result[i].availableMemory = value.availableMemory
        i = i + 1    
      #resp = offload.msg.SystemStats()
      #resp.cpuUsage = self.stats.get(node_name, default).cpuUsage
      #resp.availableMemory = self.stats.get(node_name, default).availableMemory
      return resp

if __name__ == '__main__':
    nodes = ['pi1', 'pi2', 'virtualpi1']
    rospy.init_node('blackboard_' + socket.gethostname())
    server = BlackboardCls(nodes)
    rospy.spin()
