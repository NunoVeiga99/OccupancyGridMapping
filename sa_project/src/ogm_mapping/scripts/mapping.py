#! /usr/bin/env python

import rospy
import numpy as np
import tf
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion





# --------------- INITIALIZATIONS ----------------------


n_height = 400 #n de celulas
n_width = 400 #n de celulas
resolution = 0.05 #resolucao em metros
min_angle = -np.pi # em radianos
max_angle = np.pi #em radianos
n_beams = 640

#Mapa com as probabilidades
ogm_map = np.zeros((n_height,n_width))
angle = np.linspace(min_angle, max_angle, 640)
lti_matrix = np.zeros((n_height, n_width)) 


a = np.linspace(0,n_width*resolution,num=n_width)
xi = np.vstack([a] * n_width)
print("Tamanho xi:")
print(xi.shape)
yi = np.hstack([np.transpose(a[np.newaxis])] * n_height)
print("Tamanho yi:")
print(yi.shape)
bearings = np.mgrid[0:n_width, 0:n_height, -np.pi:np.pi:640j]
print("Tamanho bearings:")
print(bearings.shape)


    

# --------------------- FUNCTIONS ----------------------


def lidar_callback(msg):
    global lidar_points
    lidar_points = msg.ranges
    #print("Lidar:")
    #print(lidar_points)


def pose_callback(msg):
    drone_pose = msg.pose
    map_with_OGM(drone_pose)



# The Inverse Range Sensor Model will return a matrix with Logaritmic Values and we have to put that values as Proabilities
# p(m|z,x) = 1-(1/(1+exp{log_odds}))
def log_to_prob(matrix):
    exp_logodds = np.exp(matrix)
    probability = 1- np.divide(1,1+exp_logodds)
    return probability


def inverse_range_sensor_model(x, y, yaw, zt):

    global bearings

    occupancy = np.zeros((n_width,n_height))

    zmax = 10.0
    alpha = 0.2
    beta = 0.009817477*2 # 2pi/640 - distance between adjacent beams

    x_mat = np.full((n_width,n_height),x) # array of x
    y_mat = np.full((n_width,n_height),y) # array of y

    r = np.sqrt(np.square(xi - x_mat),np.square(yi - y_mat)) # relative range 
    print("r :")
    print(r)
    phi = np.arctan2(yi - y_mat,xi - x_mat) - yaw # relative bearing
    print("phi :")
    print(phi)

    phi_values = np.repeat(phi[...,np.newaxis], 640, axis=2)
    bearing_array = bearings[2]
    print("bearings")
    print(bearing_array)

    k = np.argmin(np.absolute(phi_values - bearing_array), axis=2)

    print("cheguei a meio do inverse range")

    for i in range(0,n_height):
        for j in range(0,n_width):

            z = zt[k[i][j]]

            if z == float('inf') or math.isnan(z) or r[i][j] > min(zmax, z + alpha/2) or np.absolute(phi[i][j] - angle[k[i][j]]) > beta/2:
                occupancy[i][j] = 0 # Unknown, no information 
            elif z < zmax and abs(r[i][j] - z) < alpha/2:
                occupancy[i][j] = 1 # Occupied
            elif r[i][j] < z:
                occupancy[i][j] = -1 #Free
    
    return occupancy




# Lets define a mapping with Occupancy Grid Mapping
def map_with_OGM(drone_pose):
    
    global lidar_points
    global n_width, n_height, resolution, lti_matrix

    # First lets peparare auxiliar variables
    # 1) rotation about Zaxis more known as yaw
    quaternion = drone_pose.orientation
    explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    

    # 2) Using a python and ROS nav_msgs::msg::_OccupancyGrid 
    OGM = OccupancyGrid()
    # Set up this function about the header
    OGM.header.stamp = rospy.Time.now()
    OGM.header.frame_id = "map"
    OGM.header.seq = 0
    # Set up this function about the info
    OGM.info.resolution = resolution
    OGM.info.width = n_width
    OGM.info.height = n_height
    OGM.info.map_load_time = rospy.Time.now()
    # Set up the map origin
    OGM.info.origin.position.x = 0
    OGM.info.origin.position.y = 0
    
    # Lets now write the OGM algorithm to use in the
    if len(lidar_points) != 0:
        # If the vector its not empty we should update the matrix with the help of the inverse range sensor model
        # l(t,i) = l(t-1,i) + inverse range-sensor model - l0 (l0 = 0)
        # Note: the l matriz was defined in the inverse ange sensor code with enteries like  positionx, positiony, theta, lidar_points
        lti_matrix = np.add(lti_matrix, inverse_range_sensor_model(drone_pose.position.x, drone_pose.position.y, yaw, lidar_points))
        # Converter de logaritmo para probabilidade
        Prob_Matrix = log_to_prob(lti_matrix)
        # Colocar em 1 dimensao a matriz
        Prob_Matrix_1D = np.concatenate(Prob_Matrix, axis = 0)
        # Guardar as probabilidades no OGM definido em cima
        for i in range(len(Prob_Matrix_1D)):
            OGM.data.append(Prob_Matrix_1D[i] * 100) # Multiply by 100 to obtain a 0-100% probabilities
    

    # Publish a topic with the map
    OGM_publisher = rospy.Publisher('/map_new', OccupancyGrid, queue_size=1)
    OGM_publisher.publish(OGM)


#---------------------- MAIN ------------------------------------

def main():
    # declare the node name
    rospy.init_node('mapping')

    lidarSub = rospy.Subscriber('/iris_0/scan', LaserScan, lidar_callback)
    poseSub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped, pose_callback)

    rospy.spin()


if __name__ == '__main__':
    main()








#header: 
#  seq: 380
#  stamp: 
#    secs: 689
#    nsecs: 494000000
#  frame_id: "iris_0/laser_2d"
#angle_min: -3.1400001049
#angle_max: 3.1400001049
#angle_increment: 0.00982785597444
#time_increment: 0.0
#scan_time: 0.0
#range_min: 0.0799999982119
#range_max: 10.0

