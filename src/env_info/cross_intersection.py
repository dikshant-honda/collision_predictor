#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

number_of_points = 300


def plot():
    # lane centers
    plt.plot(x_horizontal_lane, y_horizontal_lane, 'r--')
    plt.plot(x_vertical_lane, y_vertical_lane, 'g--')
    plt.plot(x_turning, y_turning, 'c--')

    # boundaries
    plt.plot(x_horizontal_lane, boundaries_left_lane, 'k-')
    plt.plot(x_horizontal_lane, boundaries_right_lane, 'k-')

    plt.plot(boundaries_left_lane, y_vertical_lane, 'k-')
    plt.plot(boundaries_right_lane, y_vertical_lane, 'k-')

    plt.axis('equal')

    plt.show()

    return


# horizontal
x_horizontal_lane = np.linspace(-150, 150, number_of_points)
y_horizontal_lane = np.linspace(0, 0, number_of_points)

# vertical
x_vertical_lane = np.linspace(0, 0, number_of_points)
y_vertical_lane = np.linspace(-150, 150, number_of_points)

# turning lane
x_half_horizontal = np.linspace(-150, 0, number_of_points//2)
y_half_horizontal = np.linspace(0, 0, number_of_points//2)
x_half_vertical = np.linspace(0, 0, number_of_points//2)
y_half_vertical = np.linspace(0, 150, number_of_points//2)

x_turning = np.hstack((x_half_horizontal, x_half_vertical))
y_turning = np.hstack((y_half_horizontal, y_half_vertical))

# lane boundaries
boundaries_left_lane = np.linspace(5, 5, number_of_points)
boundaries_right_lane = np.linspace(-5, -5, number_of_points)

# plot()
