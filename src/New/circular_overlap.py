#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray

from env_info.vehicle_info import Traffic
from New.circular_noise import add_noise
from New.intersection import intersectionArea


def noise(
        vehicle1: Traffic,
        vehicle2: Traffic,
        time_horizon: float,
) -> list:
    """
    Function to extract the vehicle data and add noise to it

    args:
        vehicle1, vehicle2: pair of traffic vehicles considered for collision check
        time_horizon: future predictions time horizon
    """
    # vehicle 1 data
    vehicle_1_position = vehicle1.location
    vehicle_1_velocity = vehicle1.velocity
    vehicle_1_radius = 1.4

    # vehicle 2 data
    vehicle_2_position = vehicle2.location
    vehicle_2_velocity = vehicle2.velocity
    vehicle_2_radius = 1.4

    # predictions using constant velocity assumption, replace later with IDM
    vehicle_1_predictions_with_noise = add_noise(
        vehicle_1_position, vehicle_1_velocity, vehicle_1_radius, time_horizon)
    vehicle_2_predictions_with_noise = add_noise(
        vehicle_2_position, vehicle_2_velocity, vehicle_2_radius, time_horizon)

    return vehicle_1_predictions_with_noise, vehicle_2_predictions_with_noise


def plotter(
        vehicle1: Traffic,
        vehicle2: Traffic,
        time_horizon: float,
) -> None:
    """
    Function to visualize the circular overlap dynamically at every time step

    args:
        vehicle1, vehicle2: pair of traffic vehicles considered for collision check
        time_horizon: future predictions time horizon
    """
    fig, ax = plt.subplots()
    ax.axis('equal')

    theta = np.linspace(0, 2 * np.pi, 100)

    vehicle_1_predictions_with_noise, vehicle_2_predictions_with_noise = noise(
        vehicle1, vehicle2, time_horizon)

    for time in range(time_horizon):
        ax.clear()

        vehicle_1_centers = vehicle_1_predictions_with_noise[time][0]
        vehicle_1_size = vehicle_1_predictions_with_noise[time][1]

        vehicle_2_centers = vehicle_2_predictions_with_noise[time][0]
        vehicle_2_size = vehicle_2_predictions_with_noise[time][1]

        # plot
        ax.plot(vehicle_1_centers[0] + vehicle_1_size * np.cos(theta),
                vehicle_1_centers[1] + vehicle_1_size * np.sin(theta))

        ax.plot(vehicle_2_centers[0] + vehicle_2_size * np.cos(theta),
                vehicle_2_centers[1] + vehicle_2_size * np.sin(theta))

        plt.pause(0.5)

    plt.show()

    return


def overlap(
        vehicle1: Traffic,
        vehicle2: Traffic,
        time_horizon: float,
) -> NDArray(np.float64):
    """
    Function to compute the normalized overlap area of two interescting circles
    at every prediction time step

    args:
        vehicle1, vehicle2: pair of traffic vehicles considered for collision check
        time_horizon: future predictions time horizon
    """

    vehicle_1_predictions_with_noise, vehicle_2_predictions_with_noise = noise(
        vehicle1, vehicle2, time_horizon)

    overlap_with_time_step = []

    for time in range(time_horizon):
        vehicle_1_centers = vehicle_1_predictions_with_noise[time][0]
        vehicle_1_size = vehicle_1_predictions_with_noise[time][1]

        vehicle_2_centers = vehicle_2_predictions_with_noise[time][0]
        vehicle_2_size = vehicle_2_predictions_with_noise[time][1]

        overlap = intersectionArea(vehicle_1_centers[0], vehicle_1_centers[1], vehicle_1_size,
                                   vehicle_2_centers[0], vehicle_2_centers[1], vehicle_2_size)

        normalized_overlap = overlap / \
            (np.pi * (vehicle_1_size**2 + vehicle_2_size**2))

        overlap_with_time_step.append(time, normalized_overlap)

    return overlap_with_time_step
