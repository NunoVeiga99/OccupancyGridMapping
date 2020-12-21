#! /usr/bin/env python

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import rospy
import struct
import numpy as np
import tf
import math

import message_filters
from sensor_msgs.msg import LaserScan
# from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion

from bresenham import get_line



# --------------- INITIALIZATIONS ----------------------

n_height = 500 # y axis
n_width = 1000 # x axis
n_z = 40 # height of the 3D map in the z axis
resolution = 0.05 #Resolution in meters
z_resolution = 0.1 #Resolution of the 3D map in the z axis
min_angle = -np.pi # Min angle in radians
max_angle = np.pi # Max angle in radians
n_beams = 640 # Total number of beams

#Definition of the maps
ogm_map = np.zeros((n_height,n_width)) # 2D map
D3_ogm_map = np.zeros((n_z,n_height,n_width)) # 3D map
lidar_angles = np.linspace(min_angle, max_angle, 640) #vector of all the lidar angles

lti_matrix = np.zeros((n_height, n_width)) 
D3lti_matrix = np.zeros((n_z,n_height,n_width))

# 3D map publisher
pub = rospy.Publisher("3d_map", PointCloud2, queue_size=2)

#Map origin
x_origin = -35
y_origin = -13
z_origin = 0

# Flag to say if we want or not 3D mapping
USE_3D_MAPPING = 0
print("3D Mapping =",USE_3D_MAPPING)



# --------------------- FUNCTIONS ----------------------


# Returns (x,y) of 2D polar data
def polar_to_rect(range,teta):
    return ([range*np.cos(teta),range*np.sin(teta)])

# Mapping class
# This class defines the main algorithms responsible for 3D and 3D mapping
class Mapping(object):

    def __init__(self):

        rospy.init_node('mapping')
        self.drone_pose = Pose()
        self.lidar_points = []
        self.path = PoseArray()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = "map"
        self.path.header.seq = 0

    # Joint Callback - receives lidar and pose info if they are synced
    def callback(self,lidar_msg, pose_msg):
        aux = Pose()
        self.lidar_points = lidar_msg.ranges
        self.drone_pose = pose_msg.pose
        aux = pose_msg.pose
        aux.position.x = self.drone_pose.position.x #+ x_origin
        aux.position.y = self.drone_pose.position.y #+ y_origin
        self.path.header = pose_msg.header
        self.path.header.stamp = rospy.Time.now()
        self.path.poses.append(aux)
        

    # The Inverse Range Sensor Model will return a matrix with Logaritmic Values and we have to put that values as Proabilities
    # p(m|z,x) = 1-(1/(1+exp{log_odds}))
    def log_to_prob(self,matrix):
        exp_logodds = np.exp(matrix)
        probability = 1- np.divide(1,1+exp_logodds)
        return probability


    def inverse_range_sensor_model(self, x, y, yaw, zt):

        np.set_printoptions(threshold=np.inf)
        occupancy = np.zeros((n_height,n_width))
        print("x:",x," y:",y, " yaw:", yaw)
        zmax = 10.0
        alpha = 0.2
        beta = 0.009817477*2 # 2pi/640 - distance between adjacent beams    
            
        x_mat = -x_origin+x # in meters 
        y_mat = -y_origin+y # in meters
        x_pos = int(x_mat/resolution) # in pixels
        y_pos = int(y_mat/resolution) # in pixels
        print('Drone grid position =(',x_pos,y_pos,')')
        for i in range(n_beams):
            
            # Corresponding range            
            z = zt[i]
            # For now we will ignore the unknown positions
            # but is possible t
            if math.isnan(z): # if there was a reading error
                continue
            
            limit = True # if we reveive a inf, we know that the space between the zmax
            #and the drone is empty. So we can map it as empty
            if z == float('inf') :
                continue

                            
            #(target_x,target_y) is the position that the laser is reading, if the
            # drone is at (0,0). Further translation is necessary
            target = polar_to_rect(z,(lidar_angles[i]+yaw))
            
            # points is a list (converted to tuple for more ) of all the points that compose the beam (like a pixelized line in paint)
            points = get_line((x_pos,y_pos),(int((target[0])/resolution)+x_pos,int((target[1])/resolution)+y_pos))

                    
            for j in range(len(points)):

                if ((points[j][1] >= n_height ) or (points[j][0] >= n_width) or (points[j][0] < 0) or (points[j][1] < 0)):
                    continue
                
                z_new = resolution*math.sqrt(((points[j][0]-x_pos)*(points[j][0]-x_pos))+((points[j][1]-y_pos)*(points[j][1]-y_pos)))
                if limit and z < zmax and abs(z_new - z) < alpha/2:
                    occupancy[points[j][1]][points[j][0]] = 1 # Occupied
                elif z_new <= z:
                    occupancy[points[j][1]][points[j][0]] = -1 #Free

        return occupancy
    

    # Lets define a mapping with Occupancy Grid Mapping
    def map_with_OGM(self, lidar_points, drone_pose):
        
        global n_width, n_height, n_z, resolution, lti_matrix, D3lti_matrix, pub

        # First lets peparare auxiliar variables
        # 1) rotation about Zaxis more known as yaw
        quaternion = drone_pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        euler = (roll, pitch, yaw)
    

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
        OGM.info.origin.position.x = x_origin
        OGM.info.origin.position.y = y_origin
        #OGM.info.origin.position.z = z_origin
        
        # Lets now write the OGM algorithm to use in the
        if len(lidar_points) != 0 and abs(roll) < 0.05 and abs(pitch) < 0.05:

            # If the vector its not empty we should update the matrix with the help of the inverse range sensor model
            # l(t,i) = l(t-1,i) + inverse range-sensor model - l0 (l0 = 0)
            # Note: the l matriz was defined in the inverse range sensor code with entries like  positionx, positiony, theta, lidar_points
            #l_current =  self.inverse_range_sensor_model(drone_pose.position.x, drone_pose.position.y, yaw, lidar_points)
            l_current =  self.inverse_range_sensor_model(drone_pose.position.x, drone_pose.position.y, yaw, lidar_points)
            lti_matrix = np.add(lti_matrix,l_current)
            # Converter de logaritmo para probabilidade
            Prob_Matrix = self.log_to_prob(lti_matrix)
            # Colocar em 1 dimensao a matriz
            Prob_Matrix_1D = np.concatenate(Prob_Matrix, axis = 0)
            
            # Guardar as probabilidades no OGM definido em cima
            for i in range(len(Prob_Matrix_1D)):
                OGM.data.append(Prob_Matrix_1D[i] * 100) # Multiply by 100 to obtain a 0-100% probabilities
            
            
            # now Is the 3D part
            if(USE_3D_MAPPING):
                pixel_z =drone_pose.position.z + (z_origin) # in meters 
                pixel_z = int(pixel_z/z_resolution)
                print("z_pizel = ",pixel_z)
                print("drone z pos = ",drone_pose.position.z)
                # if the drone is in the map
                if(pixel_z < n_z and 0  <= pixel_z):
                
                    # Now for the 3D
                    D3lti_matrix[pixel_z] = np.add(D3lti_matrix[pixel_z], l_current)
                    # Converter de logaritmo para probabilidade
                    D3Prob_Matrix = self.log_to_prob(D3lti_matrix)            
                    self.publish_3D_map(D3Prob_Matrix,drone_pose.position.x, drone_pose.position.y,drone_pose.position.z)
        
        
        # Publish a topic with the map
        OGM_publisher = rospy.Publisher('/2d_map', OccupancyGrid, queue_size=1)
        OGM_publisher.publish(OGM)
        Path_publisher = rospy.Publisher('drone_path', PoseArray, queue_size = 1)
        Path_publisher.publish(self.path)

        
        
    # The development of this function is based in this question dicussed in 
    # this forum:
    # https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
        
    def publish_3D_map(self,D3Prob_Matrix,drone_x,drone_y,drone_z):
        points = []
        inc = 0
        # print("probability value = ",D3Prob_Matrix[0:3,:,:])
        for i in range(n_z):
            for j in range(n_height):
                for k in range(n_width):
                    x = (float(k)*resolution) + (x_origin)#
                    y = (float(j)*resolution) + (y_origin)#
                    z = ((float(i)*z_resolution) + (z_origin))
                    
                    # We just want borders!
                    # In the future, we might consider increase the probability upper 0.5
                    if(D3Prob_Matrix[i,j,k] <= 0.5):
                        continue
                    inc += 1
                    
                    r = int(0.1*255.0*(i+1)/(n_z+10))
                    g = int(255.0*(i+1)/(n_z+10))
                    b = int(0.1*255.0*(i+1)/(n_z+10))
                    a = 255
                    #print(r,g,b,(i+1),(n_z+10))
                        
                    # print r, g, b, a
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    # print hex(rgb)
                    pt = [x, y, z, rgb]
                    points.append(pt)
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  # PointField('rgb', 12, PointField.UINT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1),
                  ]

        # print points
        print("publishing PC2 with",inc," points")
        header = Header()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, points)
        
        # while not rospy.is_shutdown():
        pc2.header.stamp = rospy.Time.now()
        pub.publish(pc2)
        # rospy.sleep(1.0)

        


#---------------------- MAIN ------------------------------------

def main():
    # declare the node name
    rospy.init_node('mapping')  

    #Mapping object
    mapping = Mapping()
    
    #Subscribers
    lidarSub = message_filters.Subscriber('/iris_0/scan', LaserScan)
    poseSub = message_filters.Subscriber('/mavros/local_position/pose',PoseStamped)

    #ApproximateTimeSyncronizer is used to sync lidar and pose info
    ts = message_filters.ApproximateTimeSynchronizer([lidarSub, poseSub], queue_size = 1, slop = 0.05)
    ts.registerCallback(mapping.callback)
    
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        mapping.map_with_OGM(mapping.lidar_points, mapping.drone_pose)
        rate.sleep()
    


if __name__ == '__main__':
    main()



