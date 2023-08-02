#! /usr/bin/env python3

from math import acos, floor, sin, sqrt

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from IDM.frenet import Point2D
from IDM.idm import IDM, predict_trajectory
from IDM.path import Path
from New.circular_noise import add_noise


def intersectionArea(
    X1: float,
    Y1: float,
    R1: float,
    X2: float,
    Y2: float,
    R2: float,
) -> float:
    """
    calculate the overlap area of two circles

    args: 
        X1, Y1, R1: (X1, Y1) represents the center of circle1 with radius R1
        X2, Y2, R2: (X2, Y2) represents the center of circle2 with radius R2
    """

    np.pi = 3.14

    d = sqrt(((X2 - X1) * (X2 - X1)) + ((Y2 - Y1) * (Y2 - Y1)))

    if (d > R1 + R2):
        ans = 0

    elif (d <= (R1 - R2) and R1 >= R2):
        ans = floor(np.pi * R2 * R2)

    elif (d <= (R2 - R1) and R2 >= R1):
        ans = floor(np.pi * R1 * R1)

    else:
        alpha = acos(((R1 * R1) + (d * d) - (R2 * R2)) / (2 * R1 * d)) * 2
        beta = acos(((R2 * R2) + (d * d) - (R1 * R1)) / (2 * R2 * d)) * 2

        a1 = (0.5 * beta * R2 * R2) - (0.5 * R2 * R2 * sin(beta))
        a2 = (0.5 * alpha * R1 * R1) - (0.5 * R1 * R1 * sin(alpha))
        ans = floor(a1 + a2)

    return ans


def overlap(
        vehicle_1_data: list,
        vehicle_2_data: list,
) -> float:
    """
    Function to compute the normalized overlap area of two interescting circles
    at every prediction time step

    args:
        vehicle_1_data: future trajectory information with uncertainity size for vehicle 1, 
        vehicle_2_data: future trajectory information with uncertainity size for vehicle 2
    """

    vehicle_1_centers = vehicle_1_data[0]
    vehicle_1_size = vehicle_1_data[1]

    vehicle_2_centers = vehicle_2_data[0]
    vehicle_2_size = vehicle_2_data[1]

    overlap = intersectionArea(vehicle_1_centers.x, vehicle_1_centers.y, vehicle_1_size,
                               vehicle_2_centers.x, vehicle_2_centers.y, vehicle_2_size)

    normalized_overlap = overlap / \
        (np.pi * (vehicle_1_size**2 + vehicle_2_size**2))

    return normalized_overlap


def plotter(
        ax,
        vehicle_1_data: NDArray[np.float64],
        vehicle_2_data: NDArray[np.float64],
) -> None:
    """
    Function to visualize the circular overlap dynamically at every time step

    args:
        ax: plotting tool
        vehicle_1_data: future trajectory information with uncertainity size for vehicle 1, 
        vehicle_2_data: future trajectory information with uncertainity size for vehicle 2
    """

    # ax.clear()

    # visualization parameters
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)

    theta = np.linspace(0, 2 * np.pi, 100)

    vehicle_1_centers = vehicle_1_data[0]
    vehicle_1_size = vehicle_1_data[1]

    vehicle_2_centers = vehicle_2_data[0]
    vehicle_2_size = vehicle_2_data[1]

    # plot
    ax.plot(vehicle_1_centers.x + vehicle_1_size * np.cos(theta),
            vehicle_1_centers.y + vehicle_1_size * np.sin(theta), 'r')

    ax.plot(vehicle_2_centers.x + vehicle_2_size * np.cos(theta),
            vehicle_2_centers.y + vehicle_2_size * np.sin(theta), 'b')

    plt.draw()
    plt.pause(0.1)

    return


# """
# sample test code for two `non-interacting vehicles`

if __name__ == "__main__":
    # plotting tools
    fig, ax = plt.subplots()
    ax.axis('equal')

    time_horizon = 50
    time_step = 0.1

    # definining the route structure
    number_of_points = 100

    x_coords = np.linspace(0, 150, number_of_points)
    y_coords = np.linspace(0, 0, number_of_points)

    boundaries_left = np.linspace(0.5, 0.5, number_of_points)
    boundaries_right = np.linspace(-0.5, -0.5, number_of_points)

    # obtaining the path from the route
    route = Path(x_coords, y_coords)
    path = route.get_path()

    # initializations for vehicle 1
    ego_idm_1 = IDM()
    ego_position_1 = Point2D(0, 0)
    ego_speed_1 = Point2D(20, 0)
    ego_radius_1 = 0.5

    # initializations for vehicle 2
    ego_idm_2 = IDM()
    ego_position_2 = Point2D(5, 0)
    ego_speed_2 = Point2D(20, 0)
    ego_radius_2 = 0.5

    # initializations for lead vehicle at a very far distance
    lead_position = Point2D(np.inf, 0)
    lead_speed = Point2D(10, 0)

    vehicle_1_time_steps, vehicle_1_trajectory, _ = predict_trajectory(
        ego_idm_1, ego_position_1, ego_speed_1, lead_position, lead_speed, path, time_horizon, time_step)
    vehicle_2_time_steps, vehicle_2_trajectory, _ = predict_trajectory(
        ego_idm_2, ego_position_2, ego_speed_2, lead_position, lead_speed, path, time_horizon, time_step)

    vehicle_1_predictions_with_noise = add_noise(
        vehicle_1_time_steps, vehicle_1_trajectory, ego_speed_1, ego_radius_1)
    vehicle_2_predictions_with_noise = add_noise(
        vehicle_2_time_steps, vehicle_2_trajectory, ego_speed_2, ego_radius_2)

    for time in range(time_horizon):
        overlap_area = overlap(
            vehicle_1_predictions_with_noise[time], vehicle_2_predictions_with_noise[time])
        plotter(ax,
                vehicle_1_predictions_with_noise[time], vehicle_2_predictions_with_noise[time])

        print("collision probability:", overlap_area)

    plt.show()
# """
