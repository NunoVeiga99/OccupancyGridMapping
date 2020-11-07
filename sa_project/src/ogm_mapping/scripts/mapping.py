#! /usr/bin/env python

import rospy
import numpy as np
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

n_height = 400 #n de celulas
n_width = 400 #n de celulas
resolution = 0.05 #resolucao em metros
min_angle = -np.pi # em radianos
max_angle = np.pi #em radianos
#depth = ???

#Mapa com as probabilidades
map = np.zeros((n_height,n_width))
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

# Lets define a mapping with Occupancy Grid Mapping
def map_with_OGM(rotation, translation, n_height, n_width, resolution, map, x_0, y_0)
    # First let's peparare auxiliar variables
    # 1) rotation about Z-axis more known as yaw
    euler_obtained_from_quaternion = euler_from_quaternion(rotation)
    yaw = euler_obtained_from_quaternion[2]
    # 2) Defining x and y positions
    positionx = translation[0]
    positiony = translation[1]
    
    # 2) Using a python and ROS nav_msgs::msg::_OccupancyGrid 
    OGM = OccupancyGrid()
    # Set up this function about the header
    OGM_.header.stamp = rospy.Time.now()
    OGM.header.frame_id = "map"
    OGM.header.seq = 0
    # Set up this function about the info
    OGM.info.resolution = resolution
    OGM.info.width = n_width
    OGM.info.height = n_height
    OGM.info.map_load_time = rospy.Time.now()
    # Set up the map origin
    OGM.info.origin.position.x = x_0
    OGM.info.origin.position.y = y_0
    
    # Let's now write the OGM algorithm to use in the
    if len(lidar_points) != 0
        # If the vector it's not empty we should update the matrix with the help of the inverse range sensor model
        # l(t,i) = l(t-1,i) + inverse range-sensor model - l0 (l0 = 0)
        # Note: the l matriz was defined in the inverse ange sensor code with enteries like  positionx, positiony, theta, lidar_points
        IRS_Matrix = np.add(IRS_Matrix, inverse_range_sensor_mode(positionx, positiony, yaw, lidar_points))
        # Converter de logaritmo para probabilidade
        Prob_Matrix = log_to_prob(IRS_Matriz)
        # Colocar em 1 dimensão a matriz
        Prob_Matrix_1D = np.concatenate(Prob_Matrix, axis = 0)
        # Guardar as probabilidades no OGM definido em cima
        for i in range(len(Prob_Matrix_1D))
            OGM.data.append(Prob_Matrix_1D[i] * 100) # Multiply by 100 to obtain a 0-100% probabilities
    
    # Publish a topic with the map
     OGM_publisher = rospy.Publisher('/map_new', OccupancyGrid, queue_size=1)
     OGM_publisher.publish(OGM)
                                           
# Dúvidas: onde vamos buscar translação para depois termos posição x e y
#          qual a posição inicial x e y
