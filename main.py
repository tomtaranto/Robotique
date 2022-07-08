# Imports go at the top
from microbit import *
from robot_class import Robot

import math

robot = Robot()
robot.init()


def cartesian_to_polar(x, y):
    return math.sqrt(math.pow(x, 2) + math.pow(y, 2)), math.atan2(y, x)


state = 0
Distances = [0, 0, 0]
success = 0


def control(dt):
    global state, success
    global Distances
    print("position : ", robot.x, robot.y)
    if robot.distance(pin1, pin2) < 15 and state != 1 and state != 2:
        print("state : ", state)
        robot.moteurStop()
        state = 1
        sleep(100)
        return
    if state == 4:
        state += 1
    elif state == 0:
        print("main loop")
        v, t = cartesian_to_polar(1, 0.1)
        print("objectif : ", v, t)
        a = robot.control(v, t, dt)
        if not a:
            state = 0
            print("Distance Done")
            sleep(5000)
    elif state == 1:
        print("Watching new distances")
        if success == 0:
            sleep(100)
            a = robot.control_angle(0, -math.pi / 2, dt)
            Distances[0] = robot.distance(pin1, pin2)
            print("in s0 : ", a)
            if not a:
                print("s0 valid")
                success = 1
        if success == 1:
            sleep(100)
            a = robot.control_angle(0, 0, dt)
            print("in s1 : ", a)
            Distances[1] = robot.distance(pin1, pin2)
            if not a:
                print("s1 valid")
                success = 2
        if success == 2:
            sleep(100)
            a = robot.control_angle(0, math.pi / 2, dt)
            Distances[2] = robot.distance(pin1, pin2)
            print("in s2 : ", a)
            if not a:
                print("s2 valid")
                success = 0
                state = 2
        print("distances : ", Distances)
    elif state == 2:
        idx = 0
        maxi = -1
        for i in range(len(Distances)):
            print(Distances, Distances[i])
            if Distances[i] > maxi:
                maxi = Distances[i]
                idx = i
        angle = -math.pi / 2 if idx == 0 else math.pi / 2 if idx == 2 else 0.98 * math.pi
        a = robot.control_angle(0, angle, dt)
        if not a:
            print("moving like a dumb")
            robot.move(2, 0)
            sleep(200)
            robot.moteurStop()
            state = 0


    else:
        state = 0
        return


while True:
    robot.update(0.1)
    control(0.1)
    sleep(100)
