from New.circular_noise import add_noise
from New.intersection import intersectionArea

import numpy as np
import matplotlib.pyplot as plt

# replace this later 
vehicle1_pos = [0, 0]
vehicle1_vel = 2
vehicle1_radius = 2.5

vehicle2_pos = [3, 3]
vehicle2_vel = 1
vehicle2_radius = 2.5

fig, ax = plt.subplots()

noise1 = add_noise(vehicle1_pos, vehicle1_vel, vehicle1_radius, 10)
noise2 = add_noise(vehicle2_pos, vehicle2_vel, vehicle2_radius, 10)

for i in range(len(noise1)):
    print(len(noise1))
    print(len(noise1[0][i]))
    print(len(noise1[0][0]))
    # ax.plot(noise1[0][i], noise1[1][i])
    # plt.pause(1.0)

# plt.show()