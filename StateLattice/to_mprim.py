from casadi import *
from matplotlib import pyplot as plt
import numpy  as np
import pickle

inputs = 10
numberofangles = 8

def mirror_across_theta(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    M = np.array(((1, 0), (0, -1)))
    mat = R@M@np.linalg.inv(R)
    mat2 = np.eye(4)
    mat2[:2, :2] = mat
    return mat2
    
def rotate(MP, theta):
    #print(theta)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    mat2 = np.eye(4)
    mat2[:2, :2] = R
    thetavec = np.array([0,0,theta,0])
    #print(mat2, MP[1], thetavec)
    return([MP[0]+thetavec, mat2@(MP[1]+thetavec), MP[2]])
    
def to_ori(theta):
    return (theta / (np.pi*2) * numberofangles)%numberofangles
def plot(start, traj):
    x0 = [start[0], start[1], start[2], 0]
    step = traj[0]
    t = traj[1]
    pointsperinput = 300
        
    path = []
    # for i in range (points + 1):
        # F = integrator('F', 'cvodes', dae, {'tf':i/points})
        # Fk = F(x0=[0,0,0,0],p=t)
        # #print(Fk['xf']* step)
        # path.append(np.array(Fk['xf']* step))

    for i in range(inputs-1):
        for j in range (pointsperinput):
            F_small = integrator('F', 'cvodes', dae, {'tf':j/pointsperinput})
            Fk = F_small(x0=x0,p=t[i:i+2])
            #print(Fk)
            path.append(np.array(Fk['xf']* step))
        Fk = F(x0=x0,p=t[i:i+2])
        x0 = Fk['xf']
        x0[3] = 0
            
            
            
    data = np.array(path)

    data = data.reshape(((inputs-1)*pointsperinput ,4))
    #print(data.tolist())

    plt.plot(data[:, 0], data[:, 1])
    #plt.axis([-20,20,-20,20])
    #plt.show()

    

    
def calc(start, traj):
    x0 = [start[0], start[1], start[2], 0]
    step = traj[0]
    t = traj[1]
    #pointsperinput = 300
        
    path = []
    # for i in range (points + 1):
        # F = integrator('F', 'cvodes', dae, {'tf':i/points})
        # Fk = F(x0=[0,0,0,0],p=t)
        # #print(Fk['xf']* step)
        # path.append(np.array(Fk['xf']* step))

    for i in range(inputs-1):
        #for j in range (pointsperinput):
        #    F_small = integrator('F', 'cvodes', dae, {'tf':j/pointsperinput})
        #    Fk = F_small(x0=x0,p=t[i:i+2])
        #    #print(Fk)
        #    path.append(np.array(Fk['xf']* step))
        Fk = F(x0=x0,p=t[i:i+2])
        x0 = Fk['xf']
        x0[3] = 0
        path.append(np.array(Fk['xf']*np.array([step,step,1,1])))
            
            
            
    data = np.array(path)

    data = data.reshape(((inputs-1) ,4))
    #print(data.tolist())
    return data
    #plt.plot(data[:, 0], data[:, 1])    
    
    
    
    
    
    
# Declare model variables
x1 = MX.sym('x1')
x2 = MX.sym('x2')
x3 = MX.sym('x3')
x4 = MX.sym('x4')


u = MX.sym('u', 2)



x = vertcat(x1, x2, x3, x4)

xdot = vertcat(cos(x[2]) , 
               sin(x[2]) ,
               u[0] * (1-x[3]) + u[1] * x[3],
               #u.T @ interpvec(x[3]),
               #u[floor(x[3]*(inputs-1))],
               1)
               
    
#print(integrator.__doc__)    
dae = {'x':x, 'p':u, 'ode':xdot}
opts = {'tf':1}
F = integrator('F', 'cvodes', dae, opts)




f = open('test1.mprim', 'w')

f.write("resolution_m: 1.000000\n")
f.write("numberofangles: {}\n".format(numberofangles))
f.write("totalnumberofprimitives: 72\n")#todo change this


#todo generate all the primitives
MPs = pickle.load(open("MP1.p", "rb"))

MPs_2 = []
for MP in MPs:
    MPs_2.append([np.array(MP[0]), np.array(MP[1]), MP[2]])


mp_new = []

#mp_new.append(MPs[0])
for MP in MPs_2:
    mp_new.append(MP)
    if MP[0][2] != MP[1][2]: #if the segment curves
        
        mp_new.append([MP[0], 
                      mirror_across_theta(MP[0][2])@ MP[1],
                      [MP[2][0],-MP[2][1]]])


mp_new_2 = []
for theta in np.arange(0, np.pi*2, np.pi/2):
    for MP in mp_new:
        mp_new_2.append(rotate(MP, theta))

# for MP in mp_new_2:
    # plot(MP[0], MP[2])
# plt.axis([-20,20,-20,20])
# plt.show()

starttheta = 1
primid = 0

for MP in mp_new_2:
    if starttheta != MP[0][2]:
        primid = 0
    starttheta = MP[0][2]
    endtheta   = MP[1][2]
    f.write("primID: {}\n".format(primid))
    primid = primid + 1
    f.write("startangle_c: {:.0F}\n".format(to_ori(starttheta)))#TODO fix this
    data = calc(MP[0], MP[2])
    f.write("endpose_c: {:.0F} {:.0F} {:.0F}\n".format(MP[1][0],MP[1][1],to_ori(data[-1, 2])))#TODO fix this
    f.write("additionalactioncostmult: 1\n")
    f.write("intermediateposes: 10\n")
    #f.write("data {}".format(MP))
    
    f.write("{:.4F} {:.4F} {:.4F}\n".format(MP[0][0], MP[0][1], MP[0][2]%(np.pi*2)))
    
    for i in data:
        f.write("{:.4F} {:.4F} {:.4F}\n".format(i[0], i[1], i[2]%(np.pi*2)))
    


























