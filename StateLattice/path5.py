from casadi import *
from matplotlib import pyplot as plt
import numpy  as np
import pickle

inputs = 10



# Declare model variables
x1 = MX.sym('x1')
x2 = MX.sym('x2')
x3 = MX.sym('x3')
x4 = MX.sym('x4')


u = MX.sym('u', 2)
#step =  MX.sym('step')

# def interp(val, values, steps):
    # step = floor((steps-1)*val)
    # interstep = (steps-1)*val - step
    # return(values[step]*(1-interstep)+ values[step+1]*interstep)
    
# def interpvec(val):
    # a=np.arange(inputs)
    # b = a - (val*(inputs-1))
    # c = fabs(b)
    # #d = fmax(-c+1, np.zeros(inputs))
    # return d


#lut = interpolant('lut', 'linear', [u], np.arange(0,1+1/(inputs-1), 1/(inputs-1)).tolist())

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





def trajectory(start, goal, turnrate):
    startx     = start[0]
    starty     = start[1]
    starttheta = start[2]
    startcurv  = start[3]
    
    goalx      = goal[0]
    goaly      = goal[1]
    goaltheta  = goal[2]
    goalcurv   = goal[3]

    opti = casadi.Opti()
    #turn = opti.variable(steps)
    turn = opti.variable(inputs)
    stepsize = opti.variable()

    x0=[startx,starty,starttheta,0]
    x=x0
    #Fks = [F(x0=x0,p=turn[0:2])]
    for i in range(inputs-1):
        Fk = F(x0=x,p=turn[i:i+2])
        x = Fk['xf']
        x[3] = 0
        


    #print(Fk)
    opti.minimize(stepsize)
    opti.subject_to(Fk['xf'][0]*stepsize == goalx)
    opti.subject_to(Fk['xf'][1]*stepsize == goaly)
    opti.subject_to(Fk['xf'][2]          == goaltheta)
    #opti.subject_to(stepsize < np.hypot(startx-goalx, starty-goaly)/(inputs - 1)*2)
    opti.subject_to(stepsize > 0)
    
    opti.subject_to(turn < 5)

    #opti.subject_to(turn[0]       == startcurv)
    #opti.subject_to(turn[-1]      == goalcurv)
    opti.subject_to(turn/stepsize <  1)
    opti.subject_to(turn/stepsize >  -1)



    opti.set_initial(stepsize, 1)
    opti.set_initial(turn[:5], 1)
    opti.set_initial(turn[5:], -1)
    

    
    p_opts = {}
    s_opts = {'max_iter': 3000} # iteration limitation
    opti.solver('ipopt',p_opts,s_opts) # set numerical backend

    #opti.solver('ipopt')
    sol = opti.solve()

    opti.set_initial(stepsize, sol.value(stepsize))
    opti.set_initial(turn, sol.value(turn))
    opti.subject_to(turn/stepsize <  turnrate)
    opti.subject_to(turn/stepsize >  -turnrate)
    
    sol = opti.solve()

    #print('x: ', sol.value(x))
    #print(sol.value(y))
    print(sol.value(stepsize))
    #print(sol.value(theta))
    print(sol.value(turn))
    t = sol.value(turn)
    step = sol.value(stepsize)
    
    return [step, t]


'''
opti.set_initial(stepsize, sol.value(stepsize))
opti.set_initial(turn, sol.value(turn))

solvers = [
'AmplInterface',
'blocksqp',
'bonmin',
'ipopt',
'knitro',
'snopt',
'worhp',
'qrsqp',
'scpgen',
'sqpmethod']

opti.solver(solvers[4])
sol = opti.solve()
'''

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
    plt.axis([-20,20,-20,20])
    plt.show()

    
    
    
    
    
    
    
#x, y, theta, curvature
#start = [0,0,0,0]
#goal = [10,20,0,0]


#thetas = np.array(range(8))/4*np.pi-np.pi
thetas_start = [0, np.pi/4]
thetas_end   = np.arange(0, np.pi+0.1, np.pi/4)

# thetas_start = [0]
# thetas_end = [0]


curvs = [0]
a = []

searcspace = 10
for i in range (-searcspace, searcspace+1):
    for j in range (-searcspace, searcspace+1):
        a.append([i,j])

A=np.array(a)
indexlist = np.argsort(np.linalg.norm(A,axis=1))
sortedA = A[indexlist]
B=sortedA[1:, :]

MPs = []

'''
for theta_start in thetas_start:
    for theta_end in thetas_end:
        for curv_start in curvs:
            for curv_end in curvs:
   
                for pos in B:
                    print([[0,0,theta_start, curv_start], [pos[0], pos[1], theta_start + theta_end, curv_end]])
                    
                    try:
                        traj = trajectory([0,0,theta_start, curv_start], [pos[0], pos[1], theta_start + theta_end, curv_end],0.2)
                        # print(traj)
                        # plot(start, traj)
                    except RuntimeError:
                        print("test")
                        continue
                    else:
                        MPs.append([[0,0,theta_start, curv_start], [pos[0], pos[1], theta_start + theta_end, curv_end], traj])
                        break
    


    
    
    
pickle.dump( MPs, open( "MP1.p", "wb" ) )


for MP in MPs:   
    print(MP) 
    plot(MP[0], MP[2])
'''

MP = trajectory([0,0,0,0], [-1,0,0,0], 0.2)
plot([0,0,0,0], MP)






'''
traj = [1.7,np.ones(10)*0.17]
plot(start, traj)

'''
















