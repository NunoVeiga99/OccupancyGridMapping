#! /usr/bin/env python

import rospy
import numpy as np
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

n_depth = 400 #n de celulas
n_width = 400 #n de celulas
resolution = 0.05 #resolucao em metros
min_angle = -np.pi # em radianos
max_angle = np.pi #em radianos
#height = ???

#Mapa com as probabilidades
map = np.zeros((n_depth,n_width))
angle = np.linspace(min_angle, max_angle, 161)#TO DO:Confirmar 161 sera o n de beams do laser

def lidar_callback(msg):
    lidar_points = msg.ranges
    #print(lidar_points)

def pose_callback(msg):
    drone_pose = msg.pose
    print("Pose:")
    print(drone_pose.position)
    print("Orientation:")
    print(drone_pose.orientation)
    
    

# declare the node name
rospy.init_node('mapping')
lidarSub = rospy.Subscriber('/iris_0/scan', LaserScan, lidar_callback)
poseSub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, pose_callback)
rospy.spin()




# The Inverse Range Sensor Model will return a matrix with Logaritmic Values and we have to put that values as Proabilities
# p(m|z,x) = 1-(1/(1+exp{log_odds}))
def log_to_prob(matrix):
    exp_logodds = np.exp(matrix)
    probability = 1- np.divide(1,1+exp_logodds)
    return probability
