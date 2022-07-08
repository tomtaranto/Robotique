#!/usr/bin/env python3
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import animation
from motor import Motor

from datetime import datetime

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
prev_e0 = 0
prev_e1 = 0

prev_t0 = 0
prev_t1 = 0


def controller(x):
    global state

    global prev_e0
    global prev_e1

    global prev_t0
    global prev_t1

    a = 12
    b = 12

    ## INSERER ICI LE CONTROL PID
    t0 = x[0, 0]
    t1 = x[1, 0]

    # P
    P = 5
    a = P * (consigne[0, 0] - t0)
    P2 = 5
    b = P2 * (consigne[1, 0] - t1)

    # Intégrale
    I1 = 1
    a += I1 * (consigne[0, 0] - t0) * dt
    I2 = 1
    b += I2 * (consigne[1, 0] - t1) * dt

    # Derivée
    D1 = 0.9
    a += D1 * ((consigne[0, 0] - t0) - prev_e0) / dt
    D2 = 0.9
    b += D2 * ((consigne[1, 0] - t1) - prev_e1) / dt

    if abs(a) > 12:
        a = sign(a) * 12
    if abs(b) > 12:
        b = sign(b) * 12

    if ((consigne[0, 0] - x[0, 0]) / pi * 180 < 2) & (state == 0):
        state = 1
        print(datetime.now() - timer)

    prev_e0 = (consigne[0, 0] - t0)
    prev_e1 = (consigne[1, 0] - t1)

    return array([[a], [b]])

def f(u):
    dθ1=motor1.update(u[0,0])
    dθ2=motor2.update(u[1,0])
    return(array([[dθ1],[dθ2]]))

def data_gen():
    t = 0
    # Initialisation des angles
    x = array([[0],[0]])
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
        

