#!/usr/bin/env python

import sys
import rospy
from offload.srv import *
from time import time

def fib_client(n):
    #rospy.wait_for_service('fib_solver')
    try:
        fib1 = rospy.ServiceProxy('fib_solver_virtual-pi-1', FibSolver)
        fib2 = rospy.ServiceProxy('fib_solver_virtual-pi-2', FibSolver)
        fib3 = rospy.ServiceProxy('fib_solver_pi-1', FibSolver)
        for x in range(0, 10):      
            if x%3 == 0:
                print "Asking virtual-pi-1"
                start = time()
                resp1 = fib1(n)
                end = time()
                print "It took %d secs"%(end-start)
            elif x%3 == 1:
                print "Asking virtual-pi-2"
                start = time()
                resp1 = fib2(n)
                end = time()
                print "It took %d secs"%(end-start)
            else:
                print "Asking pi-1"
                start = time()
                resp1 = fib3(n)
                end = time()
                print "It took %d secs"%(end-start)
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
