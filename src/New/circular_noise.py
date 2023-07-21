#! /usr/bin/env python3

import numpy as np
from numpy.typing import NDArray

from IDM.idm import IDM, predict_trajectory


def add_noise(
        ego_position: float,
        ego_speed: float,
        lead_position: float,
        lead_speed: float,
        radius: float,
        time_horizon: int,
        time_step: float,
        growth_rate: float = 1.0,
) -> list:
    """
    function to add circular noise to the future predicted position of the 
    vehicle assuming constant speed, further replace by IDM.

    returns a list of circular noise and time step: [noise(x), noise(y), time]

    args:
        ego_position: current position of the vehicle
        ego_speed: current speed of the vehicle
        lead_position: current position of the vehicle
        lead_speed: current speed of the vehicle
        radius: noise size, initialized rouhgly by the size of the vehicle
        time_horizon: time horizon for which future trajectory is needed
        time_step: dynamics update rate
        growth_rate: growth rate of the noise due to increase in uncertainity in the future
    """

    noise = []

    idm = IDM()

    time_steps, trajectory, _ = predict_trajectory(
        idm, ego_position, ego_speed, lead_position, lead_speed, time_horizon, time_step)

    for time in range(len(time_steps)):
        if growth_rate < 1.4:
            growth_rate += ego_speed * (time/50)

        size = growth_rate * radius

        noise.append([trajectory[time], size])

    return noise
