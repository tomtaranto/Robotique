#!/usr/bin/env python3
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from motor import Motor

from datetime import datetime
from scipy.optimize import minimize,Bounds

matplotlib.use('Qt5Agg')

'''Figure'''
fig, (ax1, ax2) = plt.subplots(2,1)
fig.set_figheight(8)
fig.set_figwidth(4)
# intialize two line objects (one in each axes)
line1, = ax1.plot([], [], lw=2)
text_v = ax1.text(0.8, 0.9, 'Value : ', fontsize=10)
ax1.set_ylim(-8, 8)
ax1.set_xlim(-8, 8)
ax1.grid()

line2, = ax2.plot([], [], lw=2, color='r')
line = [line1, line2]
ax2.set_ylim(0, 4)
ax2.set_xlim(0, 10)
ax2.grid()
    

timer = 0

'''System'''
L1,L2 = 4,3
c = array([[1],[2]])
r=4
dt = 1/30

xt = [0]
yt = [0]


'''Motor Parameters'''
R = 1  # Resistance (Ohms)
L = 0.35  # Inductance (H)
J = 0.02  # Inertia of the Motor (kgm2)
B = 0.3  # Friction Coefficent of the Motor
Kb = 0.02  # Feedback constant
Kt = 0.1  # Motor Constant

motor1 = Motor(R, L, B, Kt, J, Kb, dt)
motor2 = Motor(R, L, B, Kt, J, Kb, dt)
    

consigne = array([[pi],[pi]])

state = 0

def mini(u,x):
    sum = 0
    ## INSERER ICI LE CONTROL MPC
    # Fonction à minimiser
    return sum

def controller(x):
    global state
    a = 1
    b = 1
        
    bounds=[ [-8,8] for i in range(4)] + [[0,0]]

    #construct the bounds in the form of constraints
    cons = []
    for factor in range(len(bounds)):
        lower, upper = bounds[factor]
        l = {'type': 'ineq',
             'fun': lambda x, lb=lower, i=factor: x[i] - lb}
        u = {'type': 'ineq',
             'fun': lambda x, ub=upper, i=factor: ub - x[i]}
        cons.append(l)
        cons.append(u)
        
    res = minimize(mini,[x[2,0] for i in range(5)],args = x,method = 'COBYLA',constraints=cons)
    #print(res.x)
    a = res.x[0]
    b = 0
    
    if abs(a) > 12:
        a = sign(a)*12
    if abs(b) > 12:
        b = sign(b)*12
        
    if ((consigne[0,0] - x[0,0])/pi*180 < 2) & (state == 0):
        state = 1
        print(datetime.now() - timer)
    
    return array([[a],[b]])

def f(u):
    dθ1=motor1.update(u[0,0])
    dθ2=motor2.update(u[1,0])
    return(array([[dθ1],[dθ2],[u[0,0]]]))

def data_gen():
    t = 0
    # Initialisation des angles
    x = array([[0],[0],[1]])
    # Initialisation des entrées
    u = array([[1],[1]])
    
    while True:
        t += dt
        x = x + dt*f(u)
        u = controller(x)
        
        yield t,x,u

def animate(i):
    t,x,u = i
    
    xt.append(x[0,0]%(pi*2))
    yt.append(t)
    line2.set_data(yt,xt)

    θ1=x[0,0]
    θ2=x[1,0]
    z=L1*array([[cos(θ1)],[sin(θ1)]])
    y=z+L2*array([[cos(θ1+θ2)],[sin(θ1+θ2)]])
    line1.set_data([0,z[0,0],y[0,0]],[0,z[1,0],y[1,0]])
    
    text_v.set_text('Value : ' + '{:.2f}'.format(θ1%(2*pi)/pi*180))
    
    return line + [text_v]
if __name__=='__main__':
    timer = datetime.now()
    anim = animation.FuncAnimation(fig, animate,data_gen,interval=dt*1000, blit=True)

    plt.show()
        

