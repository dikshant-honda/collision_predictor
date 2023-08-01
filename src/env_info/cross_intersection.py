#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    number_of_points = 300

    # horizontal
    x_horizontal_lane = np.linspace(-150, 150, number_of_points)
    y_horizontal_lane = np.linspace(0, 0, number_of_points)

    # vertical
    x_vertical_lane = np.linspace(0, 0, number_of_points)
    y_vertical_lane = np.linspace(-150, 150, number_of_points)

    # lane boundaries
    boundaries_left_lane = np.linspace(5, 5, number_of_points)
    boundaries_right_lane = np.linspace(-5, -5, number_of_points)

    plt.plot(x_horizontal_lane, y_horizontal_lane, 'r--')
    plt.plot(x_vertical_lane, y_vertical_lane, 'r--')

    plt.plot(x_horizontal_lane, boundaries_left_lane, 'b-')
    plt.plot(x_horizontal_lane, boundaries_right_lane, 'b-')

    plt.plot(boundaries_left_lane, y_vertical_lane, 'g-')
    plt.plot(boundaries_right_lane, y_vertical_lane, 'g-')

    plt.axis('equal')

    plt.show()