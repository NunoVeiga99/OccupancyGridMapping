############   Bresenham algorithm  #############

# This file contains the implementation of the Bresenham algoritm for
# the 2D and (in the future) for the 3D

###### For the 2D ######

# # Assumptions:
# # 1. We draw the line from left to right
# # 2. x1 < x2 and y1< y2 
# # 3. Slope of the line is between 0 and 1. We draw a line from lower left to upper right.

# import numpy as np
# from fractions import Fraction

# def plotLine(x0, y0, x1, y1):
    
#     dx = x1 - x0
#     dy = y1 - y0
#     D = 2*dy - dx
#     y = y0

#     for x in range(x0,x1):
#         print(x,",",y)
#         if D > 0:
#             y = y + 1
#             D = D - 2*dx
#         D = D + 2*dy
        
        
# def line(self, x0, y0, x1, y1):
#     rev = reversed
#     if abs(y1 - y0) <= abs(x1 - x0):
#         x0, y0, x1, y1 = y0, x0, y1, x1
#         rev = lambda x: x
#     if x1 < x0:
#         x0, y0, x1, y1 = x1, y1, x0, y0
#     leny = abs(y1 - y0)
#     for i in range(leny + 1):
#         self.set(*rev((round(Fraction(i, leny) * (x1 - x0)) + x0, (1 if y1 > y0 else -1) * i + y0)))
        
    
# function for line generation  
# def bresenham(x1,y1,x2, y2):  
  
    # m_new = 2 * (y2 - y1)  
    # slope_error_new = m_new - (x2 - x1) 
  
    # y=y1 
    # for x in range(x1,x2+1):  
      
    #     print("(",x ,",",y ,")\n")  
  
    #     # Add slope to increment angle formed  
    #     slope_error_new =slope_error_new + m_new  
  
    #     # Slope error reached limit, time to  
    #     # increment y and update slope error.  
    #     if (slope_error_new >= 0):  
    #         y=y+1
    #         slope_error_new =slope_error_new - 2 * (x2 - x1)  


# This one !!!!

def get_line(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    # if it is steep, we need to change the y and x, to adapt the algorithm
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    # the algorithm is used to run from right to left. If if it's the oposite
    # we need to swap the initial and final positions
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points



##############################################################
# Bresenham for 3D
# still needs checking
#
#https://www.geeksforgeeks.org/bresenhams-algorithm-for-3-d-line-drawing/

def Bresenham3D(x1, y1, z1, x2, y2, z2): 
    ListOfPoints = [] 
    ListOfPoints.append((x1, y1, z1)) 
    dx = abs(x2 - x1) 
    dy = abs(y2 - y1) 
    dz = abs(z2 - z1) 
    if (x2 > x1): 
        xs = 1
    else: 
        xs = -1
    if (y2 > y1): 
        ys = 1
    else: 
        ys = -1
    if (z2 > z1): 
        zs = 1
    else: 
        zs = -1
  
    # Driving axis is X-axis" 
    if (dx >= dy and dx >= dz):         
        p1 = 2 * dy - dx 
        p2 = 2 * dz - dx 
        while (x1 != x2): 
            x1 += xs 
            if (p1 >= 0): 
                y1 += ys 
                p1 -= 2 * dx 
            if (p2 >= 0): 
                z1 += zs 
                p2 -= 2 * dx 
            p1 += 2 * dy 
            p2 += 2 * dz 
            ListOfPoints.append((x1, y1, z1)) 
  
    # Driving axis is Y-axis" 
    elif (dy >= dx and dy >= dz):        
        p1 = 2 * dx - dy 
        p2 = 2 * dz - dy 
        while (y1 != y2): 
            y1 += ys 
            if (p1 >= 0): 
                x1 += xs 
                p1 -= 2 * dy 
            if (p2 >= 0): 
                z1 += zs 
                p2 -= 2 * dy 
            p1 += 2 * dx 
            p2 += 2 * dz 
            ListOfPoints.append((x1, y1, z1)) 
  
    # Driving axis is Z-axis" 
    else:         
        p1 = 2 * dy - dz 
        p2 = 2 * dx - dz 
        while (z1 != z2): 
            z1 += zs 
            if (p1 >= 0): 
                y1 += ys 
                p1 -= 2 * dz 
            if (p2 >= 0): 
                x1 += xs 
                p2 -= 2 * dz 
            p1 += 2 * dy 
            p2 += 2 * dx 
            ListOfPoints.append((x1, y1, z1)) 
    return ListOfPoints 
  
  
def main(): 
    (x1, y1, z1) = (-1, 1, 1) 
    (x2, y2, z2) = (5, 3, -1) 
    ListOfPoints = Bresenham3D(x1, y1, z1, x2, y2, z2) 
    print(ListOfPoints) 
  