#!/usr/bin/env python3
from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py
import math

def f(x,u):
    xr,yr,θr,vr=x.flatten()
    u1,u2=u.flatten()
    return (array([[vr*cos(θr)],[vr*sin(θr)],[u1],[u2]]))



ax=init_figure(-30,30,-30,30)

dt = 0.1
x = array([[0],[1],[pi/3],[1]])
u = array([[1],[0]])
Lx = 15
Ly = 7

def controller(x,t):
        
    return array([[0],[0]])

for t in arange(0,100,dt) :
    clear(ax)
    draw_tank(x)
    xa = Lx*sin(0.1*np.pi*np.arange(0,200*np.pi,0.1))
    ya = Ly*cos(0.1*np.pi*np.arange(0,200*np.pi,0.1))
    
    ax.plot(xa,ya)
    x = x+dt*f(x,u)
    u = controller(x,t)
pause(1)
