# Imports go at the top
from microbit import *
from robot_class import Robot

import math

robot = Robot()
robot.init()

state = 0
def control(dt):
    global state
    if robot.distance(pin1,pin2) < 25:
        robot.moteurStop()
        return
    if state == 0:
        a = robot.control(0,math.pi/4,dt)
        if not a :
            sleep(1000)
            state +=1
    elif state == 1:
        a= robot.control(0.5,0,dt)
        if not a :
            state += 1
    else:
        state = 0
        return

while True:
    robot.update(0.1)
    control(0.1)
    sleep(100)
