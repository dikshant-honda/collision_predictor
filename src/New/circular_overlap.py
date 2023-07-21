#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from New.circular_noise import add_noise
from New.intersection import intersectionArea


def noise(
        vehicle1_location: float,
        vehicle1_velocity: float,
        vehicle2_location: float,
        vehicle2_velocity: float,
        time_horizon: float,
        time_step: float,
) -> list:
    """
    Function to extract the vehicle data and add noise to it

    args:
        vehicle1, vehicle2: pair of traffic vehicles considered for collision check
        time_horizon: future predictions time horizon
        time_step: dynamics update rate
    """
    # vehicle 1 data
    vehicle_1_position = vehicle1_location
    vehicle_1_velocity = vehicle1_velocity
    vehicle_1_radius = 0.5

    # vehicle 2 data
    vehicle_2_position = vehicle2_location
    vehicle_2_velocity = vehicle2_velocity
    vehicle_2_radius = 0.5

    # predictions using constant velocity assumption, replace later with IDM
    vehicle_1_predictions_with_noise = add_noise(
        vehicle_1_position, vehicle_1_velocity, np.inf, 10, vehicle_1_radius, time_horizon, time_step)
    vehicle_2_predictions_with_noise = add_noise(
        vehicle_2_position, vehicle_2_velocity, np.inf, 10, vehicle_2_radius, time_horizon, time_step)

    return vehicle_1_predictions_with_noise, vehicle_2_predictions_with_noise


def plotter(
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

    ax.clear()

    # visualization parameters
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_xlim(-1, 50)
    ax.set_ylim(-1.4, 1.4)

    theta = np.linspace(0, 2 * np.pi, 100)

    vehicle_1_centers = [vehicle_1_data[0], 0]
    vehicle_1_size = vehicle_1_data[1]

    vehicle_2_centers = [vehicle_2_data[0], 0]
    vehicle_2_size = vehicle_2_data[1]

    # plot
    ax.plot(vehicle_1_centers[0] + vehicle_1_size * np.cos(theta),
            vehicle_1_centers[1] + vehicle_1_size * np.sin(theta), 'r')

    ax.plot(vehicle_2_centers[0] + vehicle_2_size * np.cos(theta),
            vehicle_2_centers[1] + vehicle_2_size * np.sin(theta), 'b')

    plt.draw()
    plt.pause(0.1)
    
    return


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

    vehicle_1_centers = [vehicle_1_data[0], 0]
    vehicle_1_size = vehicle_1_data[1]

    vehicle_2_centers = [vehicle_2_data[0], 0]
    vehicle_2_size = vehicle_2_data[1]

    overlap = intersectionArea(vehicle_1_centers[0], vehicle_1_centers[1], vehicle_1_size,
                               vehicle_2_centers[0], vehicle_2_centers[1], vehicle_2_size)

    normalized_overlap = overlap / \
        (np.pi * (vehicle_1_size**2 + vehicle_2_size**2))

    return normalized_overlap


if __name__ == "__main__":

    fig, ax = plt.subplots()
    ax.axis('equal')

"""
sample test code for two `non-interacting vehicles`

    time_horizon = 30
    time_step = 0.1

    # initializations for vehicle 1
    ego_position_1 = 0
    ego_speed_1 = 20

    # initializations for vehicle 2
    ego_position_2 = 5
    ego_speed_2 = 10
 
    vehicle_1_predictions_with_noise, vehicle_2_predictions_with_noise = noise(
        ego_position_1, ego_speed_1, ego_position_2, ego_speed_2, time_horizon, time_step)

    for time in range(time_horizon):
        overlap_area = overlap(vehicle_1_predictions_with_noise[time], vehicle_2_predictions_with_noise[time])
        plotter(ax, vehicle_1_predictions_with_noise[time], vehicle_2_predictions_with_noise[time]) 

        print("collision probability:", overlap_area)

    plt.show()   
"""
