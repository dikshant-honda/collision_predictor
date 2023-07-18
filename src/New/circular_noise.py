import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray


def add_noise(
        position: NDArray[np.float64],
        velocity: float,
        radius: float,
        time_horizon: int,
        growth_rate: float = 1.0
) -> list:
    """
    function to add circular noise to the future predicted position of the 
    vehicle assuming constant velocity, further replace by IDM.

    returns a list of circular noise and time step: [noise(x), noise(y), time]

    args:
        position: current position of the vehicle
        velocity: current velcoity of the vehicle
        radius: noise size, initialized rouhgly by the size of the vehicle
        time_horizon: time forizon for which future trajectory is needed
        growth_rate: growth rate of the noise due to increase in uncertainity in the future
    """

    noise = [[], [], []]
    theta = np.linspace(0, 2*np.pi, 100)

    for time in range(time_horizon):
        # for time being consider movement only in x direction, change later with IDM
        position[0] = velocity * time

        if growth_rate < 1.4:
            growth_rate += velocity * (time/50)

        x = position[0] + growth_rate * radius * np.cos(theta)
        y = position[1] + growth_rate * radius * np.sin(theta)

        noise[0].append(x)
        noise[1].append(y)
        noise[2].append(time)

    return noise
