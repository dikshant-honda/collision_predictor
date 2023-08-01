#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from env_info.cross_intersection import *
from IDM.frenet import Point2D
from IDM.idm import IDM, predict_trajectory, time_to_collision
from IDM.path import Path
from New.circular_noise import add_noise as add_circular_noise
from New.circular_overlap import overlap as circular_overlap
from New.circular_overlap import plotter as circle_plotter
from New.elliptical_noise import add_noise as add_elliptical_noise
from New.elliptical_overlap import overlap as elliptical_overlap
from New.elliptical_overlap import plotter as ellipse_plotter


def elliptical_predictions(idm, ego_position, ego_speed, ego_major_axis, ego_minor_axis, ego_orientation, lead_position, lead_speed, lead_major_axis, lead_minor_axis, lead_orientation, path, time_horizon, time_step):
    # predict future trajectory using IDM
    time, ego_trajectory, lead_trajectory = predict_trajectory(
        idm, ego_position, ego_speed, lead_position, lead_speed, path, time_horizon, time_step)

    # add uncertainity in the predicted trajectory
    ego_predictions_with_elliptical_noise = add_elliptical_noise(
        time, ego_trajectory, ego_speed, ego_major_axis, ego_minor_axis, ego_orientation)

    lead_predictions_with_elliptical_noise = add_elliptical_noise(
        time, lead_trajectory, lead_speed, lead_major_axis, lead_minor_axis, lead_orientation)

    return ego_predictions_with_elliptical_noise, lead_predictions_with_elliptical_noise


def circular_predictions(idm, ego_position, ego_speed, lead_position, lead_speed, path, time_horizon, time_step):
    # predict future trajectory using IDM
    time, ego_trajectory, lead_trajectory = predict_trajectory(
        idm, ego_position, ego_speed, lead_position, lead_speed, path, time_horizon, time_step)

    # add uncertainity in the predicted trajectory
    ego_predictions_with_circular_noise = add_circular_noise(
        time, ego_trajectory, ego_speed, ego_size)
    lead_predictions_with_circular_noise = add_circular_noise(
        time, lead_trajectory, lead_speed, lead_size)

    return ego_predictions_with_circular_noise, lead_predictions_with_circular_noise


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
    idm_1 = IDM()
    idm_2 = IDM()

    # obtaining the path from the route
    route_1 = Path(x_horizontal_lane, y_horizontal_lane)
    path_1 = route_1.get_path()

    route_2 = Path(x_vertical_lane, y_vertical_lane)
    path_2 = route_2.get_path()

    # initializations for ego vehicle 1
    ego_position_1 = Point2D(-50, 0)
    ego_speed_1 = np.array([10, 0])
    ego_size = 0.6
    ego_major_axis_1 = 0.6
    ego_minor_axis_1 = 0.2
    ego_orientation_1 = 0.0

    lead_position_1 = Point2D(-30, 0)
    lead_speed_1 = np.array([4, 0])
    lead_size = 0.6
    lead_major_axis_1 = 0.6
    lead_minor_axis_1 = 0.2
    lead_orientation_1 = 0

    # initializations for ego vehicle 1
    ego_position_2 = Point2D(0, -50)
    ego_speed_2 = np.array([0, 10])
    ego_size = 0.6
    ego_major_axis_2 = 0.6
    ego_minor_axis_2 = 0.2
    ego_orientation_2 = 90.0

    lead_position_2 = Point2D(0, -20)
    lead_speed_2 = np.array([0, 4])
    lead_size = 0.6
    lead_major_axis_2 = 0.6
    lead_minor_axis_2 = 0.2
    lead_orientation_2 = 90.0

    for step in range(sim_time):
        ax.clear()

        # visualizing the road boundaries
        ax.plot(x_horizontal_lane, y_horizontal_lane, 'r--')
        ax.plot(x_vertical_lane, y_vertical_lane, 'r--')

        ax.plot(x_horizontal_lane, boundaries_left_lane, 'b-')
        ax.plot(x_horizontal_lane, boundaries_right_lane, 'b-')

        ax.plot(boundaries_left_lane, y_vertical_lane, 'g-')
        ax.plot(boundaries_right_lane, y_vertical_lane, 'g-')

        ax.axis('equal')

        print("simulation time step:", step)

        ego_predictions_with_circular_noise_1, lead_predictions_with_circular_noise_1 = circular_predictions(
            idm_1, ego_position_1, ego_speed_1, lead_position_1, lead_speed_1, path_1, time_horizon, time_step)
        ego_predictions_with_circular_noise_2, lead_predictions_with_circular_noise_2 = circular_predictions(
            idm_2, ego_position_2, ego_speed_2, lead_position_2, lead_speed_2, path_2, time_horizon, time_step)

        ego_predictions_with_elliptical_noise_1, lead_predictions_with_elliptical_noise_1 = elliptical_predictions(
            idm_1, ego_position_1, ego_speed_1, ego_major_axis_1, ego_major_axis_1, ego_orientation_1, lead_position_1, lead_speed_1, lead_major_axis_1, lead_major_axis_1, lead_orientation_1, path_1, time_horizon, time_step)
        ego_predictions_with_elliptical_noise_2, lead_predictions_with_elliptical_noise_2 = elliptical_predictions(
            idm_2, ego_position_2, ego_speed_2, ego_major_axis_2, ego_major_axis_2, ego_orientation_2, lead_position_2, lead_speed_2, lead_major_axis_2, lead_major_axis_2, lead_orientation_2, path_2, time_horizon, time_step)

        # circular overlap check
        for time in range(time_horizon):
            overlap_area_1 = circular_overlap(
                ego_predictions_with_circular_noise_1[time], lead_predictions_with_circular_noise_1[time])
            overlap_area_2 = circular_overlap(
                ego_predictions_with_circular_noise_2[time], lead_predictions_with_circular_noise_2[time])

            circle_plotter(
                ax, ego_predictions_with_circular_noise_1[time], lead_predictions_with_circular_noise_1[time])
            circle_plotter(
                ax, ego_predictions_with_circular_noise_2[time], lead_predictions_with_circular_noise_2[time])

            if overlap_area_1 > 0.1:
                print("collision probability for ego 1:", overlap_area_1,
                      "after:", time*time_step, "seconds!")

            if overlap_area_2 > 0.1:
                print("collision probability for ego 2:", overlap_area_2,
                      "after:", time*time_step, "seconds!")

        # elliptical overlap check
        for time in range(time_horizon):
            overlap_area_1 = elliptical_overlap(
                ego_predictions_with_elliptical_noise_1[time], lead_predictions_with_elliptical_noise_1[time])
            overlap_area_2 = elliptical_overlap(
                ego_predictions_with_elliptical_noise_2[time], lead_predictions_with_elliptical_noise_2[time])

            ellipse_plotter(
                ax, ego_predictions_with_elliptical_noise_1[time], lead_predictions_with_elliptical_noise_1[time])
            ellipse_plotter(
                ax, ego_predictions_with_elliptical_noise_2[time], lead_predictions_with_elliptical_noise_2[time])

            if overlap_area_1 > 0.1:
                print("collision probability for ego 1:", overlap_area_1,
                      "after:", time*time_step, "seconds!")

            if overlap_area_2 > 0.1:
                print("collision probability for ego 2:", overlap_area_2,
                      "after:", time*time_step, "seconds!")

        # time to collision evaluation
        TTC_1 = time_to_collision(
            ego_position_1.x, ego_speed_1[0], lead_position_1.x, lead_speed_1[0])
        print("time to collision:", TTC_1, "seconds!")

        TTC_2 = time_to_collision(
            ego_position_2.y, ego_speed_2[1], lead_position_1.y, lead_speed_1[1])
        print("time to collision:", TTC_2, "seconds!")

        # take a step in the real world
        ego_position_1.x += ego_speed_1[0] * time_move
        lead_position_1.x += lead_speed_1[0] * time_move

        ego_position_2.y += ego_speed_2[1] * time_move
        lead_position_2.y += lead_speed_2[1] * time_move

        print("-------------------------------------")

    plt.show()
