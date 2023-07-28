#! /usr/bin/env python3

import numpy as np
from numpy.typing import NDArray


def add_noise(
        time_steps: NDArray[np.float64],
        trajectory: NDArray[np.float64],
        velocity: NDArray[np.float64],
        major_axis: float,
        minor_axis: float,
        orientation: float,
        growth_rate: float = 1.0,
) -> list:
    """
    function to add circular noise to the future predicted position using IDM

    returns a list of circular noise and time step: [noise(trajectory), time]

    args:
        time_steps: predicted trajectory time steps
        trajectory: future trajectory estimated using IDM
        velocity: current velocity of the vehicle
        major_axis: length of major axis, initialized by the length of the car
        minor_axis: length of minor axis, initialized by the width of the car
        orientation: orientation of the vehicle, theta
        growth_rate: growth rate of the noise due to increase in uncertainity in the future
    """

    noise = []

    speed = np.sqrt(velocity[0] ** 2 + velocity[1] ** 2)

    for time in range(len(time_steps)):
        if growth_rate < 1.4:
            growth_rate += speed * (time/50)

        new_major_axis = growth_rate * major_axis
        new_minor_axis = growth_rate * minor_axis

        new_ellipse_params = [new_major_axis, new_minor_axis, orientation]

        noise.append([trajectory[time], new_ellipse_params])

    return noise
