# -*- coding: utf-8 -*-

import numpy as np
import math

# --------------- INITIALIZATIONS ----------------------

lidar_points = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]


n_height = 10 #n de celulas
n_width = 10 #n de celulas
resolution = 0.5 #resolucao em metros
min_angle = -np.pi # em radianos
max_angle = np.pi #em radianos
n_beams = 10

#Mapa com as probabilidades
ogm_map = np.zeros((n_height,n_width))
lidar_angles = np.linspace(min_angle, max_angle, n_beams)
print("LIDAR ANGLES:")
print(lidar_angles)

lti_matrix = np.zeros((n_height, n_width)) 

a = np.linspace(-2.5,-2.5+n_width*resolution,num=n_width)
xi = np.vstack([a] * n_width)
print("xi:")
print(xi)
yi = np.hstack([np.transpose(a[np.newaxis])] * n_height)
print("yi:")
print(yi)


#%% Inverse Range Sensor Model

x = 0
y = 0
yaw = 0

occupancy = np.zeros((n_width,n_height))

zmax = 3.0
alpha = 0.2
beta = (np.pi/10)*2 # 2pi/640 - distance between adjacent beams

x_mat = np.full((n_width,n_height),x) # array of x
y_mat = np.full((n_width,n_height),y) # array of y

r = np.sqrt(np.square(xi - x_mat) + np.square(yi - y_mat))
print("RRRR: \n", r)

phi = np.arctan2(yi - y_mat,xi - x_mat) - yaw
print("PHI PHI: \n", phi)


for i in range(0,n_height):
        for j in range(0,n_width):

            k = np.argmin(np.absolute(lidar_angles - phi[i][j]), axis=-1)
            #print("LIDAR - PHI:")
            #print(lidar_angles - phi[i][j])
            z = lidar_points[k] 
            #print("closest point:")
            #print(z)
            #print("Index of closest angle")
            #print(k)

            if z == float('inf') or math.isnan(z) or r[i][j] > min(zmax, z + alpha/2) or np.absolute(phi[i][j] - lidar_angles[k]) > beta/2:
                occupancy[i][j] = 0 # Unknown, no information 
            elif z < zmax and abs(r[i][j] - z) < alpha/2:
                occupancy[i][j] = 1 # Occupied
            elif r[i][j] < z:
                occupancy[i][j] = -1 #Free
                
            #print("map", i*10 + j , occupancy)


print("MAP FINAL:", occupancy)













def log_to_prob(matrix):
    exp_logodds = np.exp(matrix)
    probability = 1- np.divide(1,1+exp_logodds)
    return probability


def inverse_range_sensor_model(x, y, yaw, zt):

    global bearings

    occupancy = np.zeros((n_width,n_height))

    print("x:",x," y:",y, " yaw:", yaw)

    zmax = 10.0
    alpha = 0.2
    beta = 0.009817477*2 # 2pi/640 - distance between adjacent beams

    x_mat = np.full((n_width,n_height),x) # array of x
    y_mat = np.full((n_width,n_height),y) # array of y

    r = np.sqrt(np.square(xi - x_mat),np.square(yi - y_mat)) # relative range 
    #print("r :")
    #print(r)
    phi = np.arctan2(yi - y_mat,xi - x_mat) - yaw # relative bearing
    #print("phi :")
    #print(phi)



    print("cheguei a meio do inverse range")
    

    for i in range(0,n_height):
        for j in range(0,n_width):

            k = np.argmin(np.absolute(lidar_angles - phi[i][j]), axis=-1)            
            #print("LIDAR - PHI:")
            #print(lidar_angles - phi[i][j])
            z = zt[k] 
            #print("closest point:")
            #print(z)
            #print("Index of closest angle")
            #print(k)

            if z == float('inf') or math.isnan(z) or r[i][j] > min(zmax, z + alpha/2) or np.absolute(phi[i][j] - lidar_angles[k]) > beta/2:
                occupancy[i][j] = 0 # Unknown, no information 
            elif z < zmax and abs(r[i][j] - z) < alpha/2:
                occupancy[i][j] = 1 # Occupied
            elif r[i][j] < z:
                occupancy[i][j] = -1 #Free
    
    return occupancy
