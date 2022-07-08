# Imports go at the top
from microbit import *
from robot_class import Robot

import math

robot = Robot()
robot.init()

def cartesian_to_polar(x,y):
    return math.sqrt(math.pow(x,2)+math.pow(y,2)), math.atan2(y,x)


state = 0
def control(dt):
    global state
    print("position : ", robot.x, robot.y)
    if robot.distance(pin1,pin2) < 25:
        robot.moteurStop()
        return
    # if state == 1:
    #     a = robot.control(1,math.pi/4,dt)
    #     if not a :
    #         sleep(1000)
    #         state +=1
    #         print("First angle Done")
    elif state == 0:
        v,t = cartesian_to_polar(1,1)
        a= robot.control(v,t,dt)
        if not a :
            state += 1
            print("Distance Done")
            sleep(5000)
    else:
        state = 0
        return

while True:
    robot.update(0.1)
    control(0.1)
    sleep(100)
