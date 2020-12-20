############   Bresenham algorithm  #############

# This file contains the implementation of the Bresenham algoritm for
# the 2D

###### For the 2D ######

# # Assumptions:
# # 1. We draw the line from left to right
# # 2. x1 < x2 and y1< y2 
# # 3. Slope of the line is between 0 and 1. We draw a line from lower left to upper right.

# However, the algorithm bellow deals with every 2D exception

# The source is https://gist.github.com/Siyeong-Lee/fdacece959c6973603dbda955e45637b

def get_line(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
    
    These are testing examples
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
    # From this point foward, the "new line" has |slope|<1
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    """ Swap (start) and (end) points if necessary and store swap state.
    # The algorithm is used to run from left to right. If we are requesting
    # to run in the oposite direction, we swapp the start and the end to 
    # let the algorithm run from "left to right", to run normaly.
    # This can be interpreted as a "reflexion" over the y axis
    # If we swapp, in the end, we swapp the final list of points"""
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials so that we can run the algorithm
    dx = x2 - x1
    dy = y2 - y1
    
 
    # Calculate error
    error = int(dx / 2.0)
    # if the slope is negative, the algorithm will instead decide if he 
    # maintains y or if he decreases y. The question is if he sums +1 or -1
    # this change in step can be visually interpreted as a "reflexion" over the 
    # x axis
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

