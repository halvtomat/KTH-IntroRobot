#!/usr/bin/env python3

"""
    # {Daniel Gustafsson}
    # {danielg8@kth.se}
"""

from math import atan2, acos, cos, sin

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]

    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    q1 = acos((pow((x - l0), 2) + pow(y, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2))
    q0 = atan2(y, (x - l0)) - atan2(l2 * sin(q1), l1 + l2 * cos(q1))
    q2 = z

    q = [q0 ,q1 ,q2]

    return q

def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """

    return q
