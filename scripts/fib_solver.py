#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt32
from time import time

def fib(n):
   if n == 1:
      return 1
   elif n == 0:   
      return 0            
   else:                      
      return fib(n-1) + fib(n-2)

def callback(data):
    start = time()
    result = fib(data.data)
    end = time()
    rospy.loginfo(rospy.get_caller_id() + "| Fib(%d) is %d and it took me %d seconds to calculate", data.data, result, end-start)
    
def fib_solver():
    rospy.init_node('fib_solver', anonymous=True)

    rospy.Subscriber("fib_tasks", UInt32, callback)

    rospy.spin()

if __name__ == '__main__':
    fib_solver()
