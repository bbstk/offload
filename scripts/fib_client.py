#!/usr/bin/env python

import sys
import rospy
from offload.srv import *

def fib_client(n):
    rospy.wait_for_service('fib_solver')
    try:
        fib = rospy.ServiceProxy('fib_solver', FibSolver)
        resp1 = fib(n)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [n]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        n = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting fib(%s)"%(n)
    print "fib(%s) = %s"%(n, fib_client(n))
