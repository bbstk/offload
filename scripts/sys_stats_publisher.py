#!/usr/bin/env python
import rospy
import offload.msg
import socket
import psutil

def reportStats():
    pub = rospy.Publisher('stats_' + socket.gethostname(), offload.msg.SystemStats)
    rospy.init_node('stats_publisher_' + socket.gethostname(), anonymous=True)
    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        msg = offload.msg.SystemStats()
        msg.name = socket.gethostname()
        msg.cpuUsage = psutil.cpu_percent(interval=1)
        msg.availableMemory = (psutil.virtual_memory().available) / 1024 / 1024
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        reportStats()
    except rospy.ROSInterruptException:
        pass
