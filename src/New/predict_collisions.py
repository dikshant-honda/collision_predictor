#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from IDM.frenet import Point2D
from IDM.idm import IDM, predict_trajectory, time_to_collision
from IDM.path import Path
from New.circular_noise import add_noise
from New.circular_overlap import overlap, plotter

if __name__ == "__main__":

    fig, ax = plt.subplots()
    ax.axis('equal')

    # time params for computing the future trajectory
    time_horizon = 50
    time_step = 0.1

    # step in real world
    time_move = 1
    sim_time = 10

    # calling the IDM class object
    idm = IDM()

    # definining the route structure
    number_of_points = 300

    x_coords = np.linspace(0, 300, number_of_points)
    y_coords = np.linspace(0, 0, number_of_points)

    boundaries_left = np.linspace(2.5, 2.5, number_of_points)
    boundaries_right = np.linspace(-2.5, -2.5, number_of_points)

    # obtaining the path from the route
    route = Path(x_coords, y_coords)
    path = route.get_path()

    # initializations
    ego_position = Point2D(0, 0)
    ego_speed = np.array([10, 0])
    ego_radius = 0.7

    lead_position = Point2D(50, 0)
    lead_speed = np.array([4, 0])
    lead_radius = 0.7

    for step in range(sim_time):
        ax.clear()

        # visualizing the road boundaries
        ax.plot(x_coords, y_coords, 'y--')
        ax.plot(x_coords, boundaries_left, "g")
        ax.plot(x_coords, boundaries_right, "g")
        print("simulation time step:", step)

        # main loop
        time, ego_trajectory, lead_trajectory = predict_trajectory(
            idm, ego_position, ego_speed, lead_position, lead_speed, path, time_horizon, time_step)

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
                print("collision probability:", overlap_area,
                      "after:", time*time_step, "seconds!")

        TTC = time_to_collision(
            ego_position.x, ego_speed[0], lead_position.x, lead_speed[0])
        print("time to collision:", TTC, "seconds!")

        # take a step in the real world
        ego_position.x += ego_speed[0] * time_move
        lead_position.x += lead_speed[0] * time_move

        print("-------------------------------------")

    plt.show()
