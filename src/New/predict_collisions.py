#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from IDM.idm import IDM, predict_trajectory, time_to_collision
from New.circular_noise import add_noise
from New.circular_overlap import overlap, plotter

if __name__ == "__main__":

    fig, ax = plt.subplots()
    ax.axis('equal')

    time_horizon = 50
    time_step = 0.1

    # calling the IDM class object
    idm = IDM()

    # initializations
    ego_position = 0
    ego_speed = 20
    ego_radius = 0.5

    lead_position = 50
    lead_speed = 4
    lead_radius = 0.5

    time, ego_trajectory, lead_trajectory = predict_trajectory(
        idm, ego_position, ego_speed, lead_position, lead_speed, time_horizon, time_step)

    ego_predictions_with_noise = add_noise(
        time, ego_trajectory, ego_speed, ego_radius)

    lead_predictions_with_noise = add_noise(
        time, lead_trajectory, lead_speed, lead_radius)

    for time in range(time_horizon):
        overlap_area = overlap(
            ego_predictions_with_noise[time], lead_predictions_with_noise[time])
        plotter(
            ax, ego_predictions_with_noise[time], lead_predictions_with_noise[time])

        if overlap_area > 0.1:
            print("collision probability:", overlap_area, "after:", time*time_step, "seconds!")

    TTC = time_to_collision(ego_position, ego_speed, lead_position, lead_speed)

    print("time to collision:", TTC, "seconds!")

    plt.show()
