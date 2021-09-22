#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Daniel Gustafsson}
# {danielg8}
# {danielg8@kth.se}

from dubins import *
import math

def out_of_bounds(car, xn, yn):
    if xn >= car.xub or yn >= car.yub or xn <= car.xlb or yn <= car.ylb:
        return True
    return False

def object_collision(car, xn, yn):
    for ob in car.obs:
        if math.sqrt((xn - ob[0])**2 + (yn - ob[1])**2) <= ob[2]:
            return True
    return False

def distance_to_goal(car, xn, yn):
    return math.sqrt((xn - car.xt)**2 + (yn - car.yt)**2)

def closest_border_distance(car, xn, yn):
    closest = math.inf
    closest = min(closest, abs(xn - car.xub))
    closest = min(closest, abs(xn - car.xlb))
    closest = min(closest, abs(yn - car.yub))
    closest = min(closest, abs(yn - car.ylb))
    return closest

def closest_object_distance(car, xn, yn):
    closest = math.inf
    for ob in car.obs:
        closest = min(closest, math.sqrt((xn - ob[0])**2 + (yn - ob[1])**2) - ob[2])
    return closest

def position_utility(car, xn, yn):
    utility = (min(closest_object_distance(car, xn, yn), closest_border_distance(car, xn, yn)*2) - distance_to_goal(car, xn, yn)) 
    return utility


def solution(car):

    # initial state
    x, y = car.x0, car.y0
    theta = 0
    controls = []
    times = [0]
    time = 1
    while(True):
        possible_angles = []
        utilities = []
        for i in reversed(range(1,51,10)):
            for j in range(17):
                    angle = math.pi/32 * (j - 8)
                    xn, yn, thetan, possible_path = x, y, theta, True
                    for k in range(i):
                        xn, yn, thetan = step(car, xn, yn, thetan, angle)
                        if(out_of_bounds(car, xn, yn) or object_collision(car, xn, yn)):
                            possible_path = False
                            break
                    if(possible_path):
                        utilities.append(position_utility(car, xn, yn))
                        possible_angles.append(angle)
            if(len(possible_angles) > 0):
                break
        if(len(possible_angles) == 0):
            break
        max_util = max(utilities)
        best_angle = possible_angles[utilities.index(max_util)]
        controls.append(best_angle)
        times.append(0.01 * time)
        time += 1
        x, y, theta = step(car, x, y, theta, best_angle)

        if(distance_to_goal(car, x, y) < 1.4):
            break

    return controls, times