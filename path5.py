from casadi import *
from matplotlib import pyplot as plt
import numpy  as np

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

    x0=[0,0,0,0]
    x=x0
    #Fks = [F(x0=x0,p=turn[0:2])]
    for i in range(inputs-1):
        Fk = F(x0=x,p=turn[i:i+2])
        x = Fk['xf']
        x[3] = 0
        


    print(Fk)
    opti.minimize(stepsize)
    opti.subject_to(Fk['xf'][0]*stepsize == goalx)
    opti.subject_to(Fk['xf'][1]*stepsize == goaly)
    opti.subject_to(Fk['xf'][2]          == goaltheta)
    opti.subject_to(stepsize < 5)
    opti.subject_to(turn < 5)

    #opti.subject_to(turn[0]       == startcurv)
    #opti.subject_to(turn[-1]      == goalcurv)
    opti.subject_to(turn/stepsize <  3)
    opti.subject_to(turn/stepsize >  -3)



    opti.set_initial(stepsize, 1)
    #opti.set_initial(turn, 0.17)
    

    opti.solver('ipopt')
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
    plt.axis([0,20,0,20])
    plt.show()

#x, y, theta, curvature
start = [0,0,0,0]
goal = [10,20,0,0]



try:
    traj = trajectory(start, goal,0.2)
    print(traj)
    plot(start, traj)
except RuntimeError:
    print("test")
    
'''
traj = [1.7,np.ones(10)*0.17]
plot(start, traj)

'''















