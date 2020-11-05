#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


def callback(msg):
    print len(msg.ranges)

# declare the node name
rospy.init_node('scan_lidar')
sub = rospy.Subscriber('/iris_0/scan', LaserScan, callback)
rospy.spin()

# The Inverse Range Sensor Model will return a matrix with Logaritmic Values and we have to put that values as Proabilities
# p(m|z,x) = 1-(1/(1+exp{log_odds}))
def log_to_prob(matrix)
    exp_logodds = np.exp(matrix)
    probability = 1- np.divide(1,1+exp_logodds)
    return probability
