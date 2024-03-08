#! /usr/bin/env python3

import numpy as np

from IDM.frenet import Point2D


def time_to_collision(
        position1: Point2D,
        velocity1: Point2D,
        position2: Point2D,
        velocity2: Point2D,
) -> float:
    """
    Function to compute the time to collision (TTC) between two traffic participants.

    args:
        position1, velocity1: position and velocity of the first vehicle
        position2, velocity2: position and velocity of the second vehicle
    """
    relative_position = np.array(
        (position1.x, position1.y)) - np.array((position2.x, position2.y))
    relative_velocity = np.array(
        (velocity1.x, velocity1.y)) - np.array((velocity2.x, velocity2.y))

    if np.dot(relative_position, relative_velocity) >= 0:
        return float("inf")

    time = -np.dot(relative_position, relative_velocity) / \
        np.dot(relative_velocity, relative_velocity)
    if time < 0:
        return 0

    return time
