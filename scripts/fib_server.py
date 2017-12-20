#!/usr/bin/env python

from offload.srv import *
import rospy
from time import time

def fib(n):
   if n == 1:
      return 1
   elif n == 0:
      return 0
   else:
      return fib(n-1) + fib(n-2)

def handle_fib(req):
    start = time()
    result = fib(req.n)
    end = time()
    rospy.loginfo(rospy.get_caller_id() + "| Fib(%d) is %d and it took me %d seconds to calculate", req.n, result, end-start)
    return FibSolverResponse(result)

def fib_server():
    rospy.init_node('fib_server')
    s = rospy.Service('fib_solver', FibSolver, handle_fib)
    print "Ready to calculate fib."
    rospy.spin()

if __name__ == "__main__":
    fib_server()
