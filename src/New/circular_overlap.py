from New.circular_noise import add_noise
from New.intersection import intersectionArea

import numpy as np
import matplotlib.pyplot as plt

# replace this later 
vehicle1_pos = [0, 0]
vehicle1_vel = [2, 0]
vehicle1_radius = 2.5

vehicle2_pos = [5, 3]
vehicle2_vel = [1, 0]
vehicle2_radius = 2.5

fig, ax = plt.subplots()
ax.axis('equal')

noise1 = add_noise(vehicle1_pos, vehicle1_vel, vehicle1_radius, 10)
noise2 = add_noise(vehicle2_pos, vehicle2_vel, vehicle2_radius, 10)

theta = np.linspace(0, 2*np.pi, 100)

for n in range(len(noise1)):
    x1 = noise1[n][0][0] + noise1[n][1] * np.cos(theta)
    y1 = noise1[n][0][1] + noise1[n][1] * np.sin(theta)
    x2 = noise2[n][0][0] + noise2[n][1] * np.cos(theta)
    y2 = noise2[n][0][1] + noise2[n][1] * np.sin(theta)

    ax.plot(x1, y1)
    ax.plot(x2, y2)
    plt.pause(0.1)

plt.show()