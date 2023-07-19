#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from numpy.typing import NDArray
from env_info.vehicle_info import Traffic
from New.circular_noise import add_noise
from New.intersection import intersectionArea

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

    normalized_overlap = overlap / \
        (np.pi*((noise1[time][1])**2 + (noise2[time][1])**2))

    print(overlap, normalized_overlap)

    # visualization
    ax.plot(noise1[time][0][0]+noise1[time][1]*np.cos(theta),
            noise1[time][0][1]+noise1[time][1]*np.sin(theta))
    ax.plot(noise2[time][0][0]+noise2[time][1]*np.cos(theta),
            noise2[time][0][1]+noise2[time][1]*np.sin(theta))
    plt.pause(1)

plt.show()

def plotter(
        vehicle1: Traffic, 
        vehicle2: Traffic,
        time_horizon: float,
) -> None:
    fig, ax = plt.subplots()
    ax.axis('equal')

    theta = np.linspace(0, 2 * np.pi, 100)
    
    # vehicle 1 data
    vehicle_1_position = vehicle1.location
    vehicle_1_velocity = vehicle1.velocity
    vehicle_1_radius = 1.4

    # vehicle 2 data
    vehicle_2_position = vehicle2.location
    vehicle_2_velocity = vehicle2.velocity
    vehicle_2_radius = 1.4

    # predictions using constant velocity assumption, replace later with IDM
    vehicle_1_predictions_with_noise = add_noise(vehicle_1_position, vehicle_1_velocity, vehicle_1_radius, time_horizon)
    vehicle_2_predictions_with_noise = add_noise(vehicle_2_position, vehicle_2_velocity, vehicle_2_radius, time_horizon)

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
    
    # vehicle 1 data
    vehicle_1_position = vehicle1.location
    vehicle_1_velocity = vehicle1.velocity
    vehicle_1_radius = 1.4

    # vehicle 2 data
    vehicle_2_position = vehicle2.location
    vehicle_2_velocity = vehicle2.velocity
    vehicle_2_radius = 1.4

    # predictions using constant velocity assumption, replace later with IDM
    vehicle_1_predictions_with_noise = add_noise(vehicle_1_position, vehicle_1_velocity, vehicle_1_radius, time_horizon)
    vehicle_2_predictions_with_noise = add_noise(vehicle_2_position, vehicle_2_velocity, vehicle_2_radius, time_horizon)

    overlap_with_time_step = []

    for time in range(time_horizon):
        vehicle_1_centers = vehicle_1_predictions_with_noise[time][0]
        vehicle_1_size = vehicle_1_predictions_with_noise[time][1]
        
        vehicle_2_centers = vehicle_2_predictions_with_noise[time][0]
        vehicle_2_size = vehicle_2_predictions_with_noise[time][1]

        overlap  = intersectionArea(vehicle_1_centers[0], vehicle_1_centers[1], vehicle_1_size,
                                    vehicle_2_centers[0], vehicle_2_centers[1], vehicle_2_size)
        
        normalized_overlap = overlap / (np.pi * (vehicle_1_size**2 + vehicle_2_size**2))

        overlap_with_time_step.append(time, normalized_overlap)

    return overlap_with_time_step
