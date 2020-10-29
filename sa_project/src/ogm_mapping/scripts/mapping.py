#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    print len(msg.ranges)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/iris_0/scan', LaserScan, callback)
rospy.spin()