#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from IDM.frenet import Point2D
from IDM.idm import IDM, predict_trajectory, time_to_collision
from IDM.path import Path
from New.circular_noise import add_noise as add_circular_noise
from New.circular_overlap import overlap as circular_overlap
from New.circular_overlap import plotter as circle_plotter
from New.elliptical_noise import add_noise as add_elliptical_noise
from New.elliptical_overlap import overlap as elliptical_overlap
from New.elliptical_overlap import plotter as ellipse_plotter

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
    ego_size = 0.6
    ego_major_axis = 0.6
    ego_minor_axis = 0.2
    ego_orientation = 0.0

    lead_position = Point2D(50, 0)
    lead_speed = np.array([4, 0])
    lead_size = 0.6
    lead_major_axis = 0.6
    lead_minor_axis = 0.2
    lead_orientation = 0.0

    for step in range(sim_time):
        ax.clear()

        # visualizing the road boundaries
        ax.plot(x_coords, y_coords, 'y--')
        ax.plot(x_coords, boundaries_left, "g")
        ax.plot(x_coords, boundaries_right, "g")
        print("simulation time step:", step)
        time, ego_trajectory, lead_trajectory = predict_trajectory(
            idm, ego_position, ego_speed, lead_position, lead_speed, path, time_horizon, time_step)

        ego_predictions_with_circular_noise = add_circular_noise(
            time, ego_trajectory, ego_speed, ego_size)

        ego_predictions_with_elliptical_noise = add_elliptical_noise(
            time, ego_trajectory, ego_speed, ego_major_axis, ego_minor_axis, ego_orientation)

        lead_predictions_with_circular_noise = add_circular_noise(
            time, lead_trajectory, lead_speed, lead_size)

        lead_predictions_with_elliptical_noise = add_elliptical_noise(
            time, lead_trajectory, lead_speed, lead_major_axis, lead_minor_axis, lead_orientation)

        # circular overlap check
        for time in range(time_horizon):
            overlap_area = circular_overlap(
                ego_predictions_with_circular_noise[time], lead_predictions_with_circular_noise[time])
            circle_plotter(
                ax, ego_predictions_with_circular_noise[time], lead_predictions_with_circular_noise[time])

            if overlap_area > 0.1:
                print("collision probability:", overlap_area,
                      "after:", time*time_step, "seconds!")

        # elliptical overlap check
        for time in range(time_horizon):
            overlap_area = elliptical_overlap(
                ego_predictions_with_elliptical_noise[time], lead_predictions_with_elliptical_noise[time])
            ellipse_plotter(
                ax, ego_predictions_with_circular_noise[time], lead_predictions_with_circular_noise[time])

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
