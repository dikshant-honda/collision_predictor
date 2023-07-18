from math import acos, floor, sin, sqrt

import numpy as np


def intersectionArea(
    X1: float,
    Y1: float,
    R1: float,
    X2: float,
    Y2: float,
    R2: float,
) -> float:
    """
    calculate the overlap area of two circles

    args: 
        X1, Y1, R1: (X1, Y1) represents the center of circle1 with radius R1
        X2, Y2, R2: (X2, Y2) represents the center of circle2 with radius R2
    """

    np.pi = 3.14

    d = sqrt(((X2 - X1) * (X2 - X1)) + ((Y2 - Y1) * (Y2 - Y1)))

    if (d > R1 + R2):
        ans = 0

    elif (d <= (R1 - R2) and R1 >= R2):
        ans = floor(np.pi * R2 * R2)

    elif (d <= (R2 - R1) and R2 >= R1):
        ans = floor(np.pi * R1 * R1)

    else:
        alpha = acos(((R1 * R1) + (d * d) - (R2 * R2)) / (2 * R1 * d)) * 2
        beta = acos(((R2 * R2) + (d * d) - (R1 * R1)) / (2 * R2 * d)) * 2

        a1 = (0.5 * beta * R2 * R2) - (0.5 * R2 * R2 * sin(beta))
        a2 = (0.5 * alpha * R1 * R1) - (0.5 * R1 * R1 * sin(alpha))
        ans = floor(a1 + a2)
    return ans
