import numpy as np
from numpy.typing import NDArray


def add_noise(
        position: NDArray[np.float64],
        velocity: NDArray[np.float64],
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

    noise = []

    for time in range(time_horizon):
        # change later with IDM predicted points
        future_points = [[], []]
        future_points[0].append(position[0] + velocity[0] * time)
        future_points[1].append(position[1] + velocity[1] * time)

        if growth_rate < 1.4:
            growth_rate += np.sqrt(velocity[0]**2 + velocity[1]**2) * (time/50)
    
        size = growth_rate * radius 

        noise.append([future_points, size])

    return noise
