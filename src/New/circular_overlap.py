from New.circular_noise import add_noise
from New.intersection import intersectionArea

import numpy as np
import matplotlib.pyplot as plt

# replace this later 
vehicle1_pos = [0, 0]
vehicle1_vel = [2, 0]
vehicle1_radius = 2.5

vehicle2_pos = [7, 4]
vehicle2_vel = [1, 0]
vehicle2_radius = 2.5

time_horizon = 10

noise1 = add_noise(vehicle1_pos, vehicle1_vel, vehicle1_radius, time_horizon)
noise2 = add_noise(vehicle2_pos, vehicle2_vel, vehicle2_radius, time_horizon)

theta = np.linspace(0, 2*np.pi, 100)

fig, ax = plt.subplots()
ax.axis('equal')

for time in range(time_horizon):
    ax.clear()
    overlap = intersectionArea(noise1[time][0][0], noise1[time][0][1], noise1[time][1],
                     noise2[time][0][0], noise2[time][0][1], noise2[time][1])
    
    normalized_overlap = overlap / (np.pi*((noise1[time][1])**2 + (noise2[time][1])**2))

    print(overlap, normalized_overlap)

    # visualization
    ax.plot(noise1[time][0][0]+noise1[time][1]*np.cos(theta), noise1[time][0][1]+noise1[time][1]*np.sin(theta))
    ax.plot(noise2[time][0][0]+noise2[time][1]*np.cos(theta), noise2[time][0][1]+noise2[time][1]*np.sin(theta))
    plt.pause(1)

plt.show()