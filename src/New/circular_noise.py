#! /usr/bin/env python3

import numpy as np
from numpy.typing import NDArray

from IDM.frenet import Point2D


def add_noise(
        time_steps: NDArray[np.float64],
        trajectory: NDArray[np.float64],
        velocity: Point2D,
        radius: float,
        growth_rate: float = 1.0,
) -> list:
    """
    function to add circular noise to the future predicted position using IDM

    returns a list of circular noise and time step: [noise(trajectory), time]

    args:
        time_steps: predicted trajectory time steps
        trajectory: future trajectory estimated using IDM
        velocity: current velocity of the vehicle
        radius: noise size, initialized roughly by the size of the vehicle
        growth_rate: growth rate of the noise due to increase in uncertainity in the future
    """

    noise = []

    speed = np.sqrt(velocity.x ** 2 + velocity.y ** 2)

    for time in range(len(time_steps)):
        if growth_rate < 1.4:
            growth_rate += speed * (time/50)

        size = growth_rate * radius

        noise.append([trajectory[time], size])

    return noise
