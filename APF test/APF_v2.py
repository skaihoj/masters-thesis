from __future__ import division
from __future__ import absolute_import
from matplotlib import pyplot as plt
import numpy as np
import cv2
from statistics import median
import math
from itertools import izip




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
    for A, B in izip(obs, obs[1:] + [obs[0]]): #Go through each pair of points 
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
        
def obstaclesFromImage(imageName):
    image = cv2.imread(imageName)
    image = image[...,0]
    obs = []
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            if image[i,j] == 0:
                obs.append([[i,j],[i,j+1],[i+1,j+1],[i+1,j]])
    return obs




'''
obs1 = [[100,100],
        [100,200],
        [200,200],
        [200,100]]
obs2 = [[250,100],
        [250,200],
        [300,200],
        [300,100]]

obstacles = [obs1, obs2]
'''

#obstacles = obstaclesFromImage("cave.png")
obstacles = [[[64, 2], [70, 47], [145, 48], [152, 4]], [[219, 56], [290, 63], [291, 128], [218, 128]], [[497, 208], [404, 199], [422, 178], [374, 148], [342, 194], [225, 200], [224, 256], [301, 259], [345, 285], [358, 267], [497, 279]], [[499, 412], [392, 412], [387, 496], [497, 499]], [[323, 376], [219, 382], [221, 329], [322, 326]], [[127, 355], [78, 352], [76, 278], [125, 280]], [[154, 176], [104, 179], [97, 115], [152, 109]]]


#[[[60, 5], [68, 50], [149, 48], [152, 6]], [[218, 56], [218, 125], [296, 131], [291, 63]], [[497, 209], [411, 200], [424, 178], [376, 150], [344, 193], [221, 201], [222, 257], [298, 259], [341, 284], [359, 270], [498, 280]], [[497, 412], [390, 412], [388, 497], [497, 497]], [[326, 377], [218, 380], [221, 330], [322, 327]], [[125, 355], [83, 354], [77, 278], [126, 278]], [[153, 175], [103, 179], [99, 113], [153, 110]]]



size = 500

#start = [0,0]
#goal = [300, 250]

start = [400,350]
goal = [300, 50]

field = np.zeros((size,size)) #for drawing direction
field2 = np.ones((size,size, 3))/1.5 #for drawing the path


'''
#Compute the direction for each pixel 
for i in xrange(size):
   for j in xrange(size):
       potential = GetGradientVector(obstacles,[i,j], goal)
       field[j,i] = math.atan2(potential[1], potential[0])
'''
       
       
#draw the obstacles       
for o in obstacles:
    cv2.fillConvexPoly(field2, np.array(o), [0,0,0])
  

#Simulate moving around the potential field
pos = np.array(start)        
for i in xrange(1000):
    dir = potential = GetGradientVector(obstacles, pos, goal)
    dir = dir / np.sqrt(np.sum(dir ** 2))
    cv2.line(field2, tuple(np.int_(pos)), tuple(np.int_(pos + dir)), [0,1,0])
    pos = pos + dir
    
    
    
    
#plt.imshow(field, cmap = u'hsv')
#plt.show()
plt.imshow(field2)
plt.show()    

