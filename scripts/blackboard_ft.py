#!/usr/bin/env python
import rospy
import offload.msg
from offload.srv import *

import socket

class BlackboardFTCls:
    stats = {}
    subscribers = []
    def __init__(self, nodes):
        for node in nodes:
          self.subscribers.append(rospy.Subscriber("stats_" + node, offload.msg.SystemStats, self.record_stats, (node), queue_size=10))
        self.reporter = rospy.Service('blackboard_ft_' + socket.gethostname(), BlackboardFT, self.report)
    
    def record_stats(self, data, node_name):
      #rospy.loginfo("Storing data for %s", node_name)
      to_store = offload.msg.SystemStats()
      to_store.name = data.name
      to_store.cpuUsage = data.cpuUsage
      to_store.availableMemory = data.availableMemory
      self.stats[node_name] = to_store

    def report(self, request):
      node_name = request.nodeName
      remove = request.remove
      resp = offload.srv.BlackboardResponse()

      if remove:
        del self.stats[node_name]

      default = offload.msg.SystemStats()
      default.cpuUsage = 100.0
      default.availableMemory = 0      
      i = 0
      for key, value in self.stats.items():
        resp.result.append(offload.msg.SystemStats())
        resp.result[i].name = value.name
        resp.result[i].cpuUsage = value.cpuUsage
        resp.result[i].availableMemory = value.availableMemory
        i = i + 1    

      return resp

if __name__ == '__main__':
    nodes = ['pi1', 'pi2','pi3','pi4','pi5','virtualpi1']
    rospy.init_node('blackboard_ft_' + socket.gethostname())
    server = BlackboardFTCls(nodes)
    rospy.spin()
