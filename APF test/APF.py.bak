from matplotlib import pyplot as plt
import numpy as np
import cv2
from statistics import median
import math




def GetClosestPointOf2(A, B, P):
    #Find the point on the line |AB| that is closest to P

    a_to_p = [P[0] - A[0], P[1] - A[1]]     # Storing vector A->P
    a_to_b = [B[0] - A[0], B[1] - A[1]]     # Storing vector A->B

    atb2 = a_to_b[0]**2 + a_to_b[1]**2  # **2 means "squared"
                                        #   Basically finding the squared magnitude
                                        #   of a_to_b

    atp_dot_atb = a_to_p[0]*a_to_b[0] + a_to_p[1]*a_to_b[1]
                                        # The dot product of a_to_p and a_to_b

    t = atp_dot_atb / atb2              # The normalized "distance" from a to
                                        #   your closest point

    x= median([A[0] + a_to_b[0]*t, A[0], B[0]]) 
    y= median([A[1] + a_to_b[1]*t, A[1], B[1]])
    return [x,y]
    #return Point.new( :x => A.x + a_to_b[0]*t,
    #                  :y => A.y + a_to_b[1]*t )
    # Add the distance to A, moving
    #   towards B

def GetClosestPoint(obs, P):
    #For the obstacle defined by obs, return the point within it closest to P
    points = []
    for A, B in zip(obs, obs[1:] + [obs[0]]): #Go through each pair of points 
                                              #|AB|, |BC|, |CD|, |DA|
        #print(A, B)
        point = GetClosestPointOf2(A, B, P)
        distance = (P[0] - point[0])**2+ (P[1] - point[1])**2 #compute the squared distance
                                                              #don't take the square root, 
                                                              #all we need it for is sorting
        points.append([distance, point])
        
    return min(points)[1]
    
def GetGradientVector(obstacles, P, goal):
    #
    potential = [0,0]
    #start with 0 potential
    for obs in obstacles:
        #add the potential for each obstacle
        point = GetClosestPoint(obs, P)
        dist = math.sqrt((P[0] - point[0])**2+ (P[1] - point[1])**2)
        if dist == 0:
            return [0,0]
        a = 1.44*10
        b = 0.4
        dgauss = -b*dist*math.e**-((dist/a)**2)
        #compute the derivative of the gaussian distribution
        potential[0] += dgauss*(point[0]-P[0])/dist
        potential[1] += dgauss*(point[1]-P[1])/dist
        #Add it to the potential
    
    goalvec = np.array(goal) - np.array(P)
    goalvec = goalvec / np.sqrt(np.sum(goalvec ** 2))
    #Add a force pulling towards the goal
    
    return potential + goalvec
        



obs1 = [[100,100],
        [100,200],
        [200,200],
        [200,100]]
obs2 = [[250,100],
        [250,200],
        [300,200],
        [300,100]]

obstacles = [obs1, obs2]
start = [0,0]
goal = [300, 250]
        
field = np.zeros((400,400)) #for drawing direction
field2 = np.zeros((400,400, 3)) #for drawing the path


#Compute the direction for each pixel 
for i in range(400):
   for j in range(400):
       potential = GetGradientVector(obstacles,[i,j], goal)
       field[j,i] = math.atan2(potential[1], potential[0])

       
       
#draw the obstacles       
for o in obstacles:
    cv2.fillConvexPoly(field2, np.array(o), [0,0,1])
    

#Simulate moving around the potential field
pos = np.array(start)        
for i in range(1000):
    dir = potential = GetGradientVector(obstacles, pos, goal)
    dir = dir / np.sqrt(np.sum(dir ** 2))
    cv2.line(field2, tuple(np.int_(pos)), tuple(np.int_(pos + dir)), [1,0,0])
    pos = pos + dir
    
    
    
    
plt.imshow(field, cmap = 'hsv')
plt.show()
plt.imshow(field2)
plt.show()    

