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


class Vehicle:
    def __init__(
            self,
            idm: IDM,
            route: Path,
            position: Point2D,
            velocity: Point2D,
            size: float,
            major_axis: float,
            minor_axis: float,
            orientation: float,
    ) -> None:
        """
        Vehicle class for capturing the vehicle dynamics

        args:
            idm: IDM class object declaration
            route: route on which the vehicle wants to follow
            position: current position of the vehicle on the route
            velocity: current velocity of the vehicle
            size: length of the vehicle, considered for determining the circular uncertatinity
            major_axis: longitudnal length of the vehicle, considered for determining the elliptical uncertainity
            minor_axis: lateral length of the vehicle, considered for determining the elliptical uncertainity
            orientation: orientation / yaw of the vehicle w.r.t. real world
        """
        self.idm = idm
        self.route = route
        self.position = position
        self.velocity = velocity
        self.size = size
        self.major_axis = major_axis
        self.minor_axis = minor_axis
        self.orientation = orientation


def elliptical_predictions(
        ego: Vehicle,
        lead: Vehicle,
        time_horizon: float,
        time_step: float,
):
    """
    function to get IDM based future predictions with elliptical uncertainity

    args:
        ego: ego vehicle information
        lead: lead vehicle information
        time_horizon: duration over which you want to predict the trajectory
        time_step: discrete interval at which you update the state variables of the system during the trajectory prediction 
    """

    path = ego.route.get_path()

    # predict future trajectory using IDM
    time, ego_trajectory, lead_trajectory = predict_trajectory(
        ego.idm, ego.position, ego.velocity, lead.position, lead.velocity, path, time_horizon, time_step)

    # add uncertainity in the predicted trajectory
    ego_predictions_with_elliptical_noise = add_elliptical_noise(
        time, ego_trajectory, ego.velocity, ego.major_axis, ego.minor_axis, ego.orientation)

    lead_predictions_with_elliptical_noise = add_elliptical_noise(
        time, lead_trajectory, lead.velocity, lead.major_axis, lead.minor_axis, lead.orientation)

    return ego_predictions_with_elliptical_noise, lead_predictions_with_elliptical_noise


def circular_predictions(
        ego: Vehicle,
        lead: Vehicle,
        time_horizon: float,
        time_step: float,
):
    """
    function to get IDM based future predictions with circular uncertainity

    args:
        ego: ego vehicle information
        lead: lead vehicle information
        time_horizon: duration over which you want to predict the trajectory
        time_step: discrete interval at which you update the state variables of the system during the trajectory prediction 
    """

    path = ego.route.get_path()

    # predict future trajectory using IDM
    time, ego_trajectory, lead_trajectory = predict_trajectory(
        ego.idm, ego.position, ego.velocity, lead.position, lead.velocity, path, time_horizon, time_step)

    # add uncertainity in the predicted trajectory
    ego_predictions_with_circular_noise = add_circular_noise(
        time, ego_trajectory, ego.velocity, ego.size)
    lead_predictions_with_circular_noise = add_circular_noise(
        time, lead_trajectory, lead.velocity, lead.size)

    return ego_predictions_with_circular_noise, lead_predictions_with_circular_noise


def get_vehicle_info():
    # calling the IDM class object
    idm_1 = IDM()
    idm_2 = IDM()

    # obtaining the path from the route
    route_1 = Path(x_horizontal_lane, y_horizontal_lane)
    route_2 = Path(x_vertical_lane, y_vertical_lane)

    # initializations for ego vehicle 1
    ego_position_1 = Point2D(-50, 0)
    ego_speed_1 = Point2D(10, 0)
    ego_size = 0.6
    ego_major_axis_1 = 0.6
    ego_minor_axis_1 = 0.2
    ego_orientation_1 = 0.0

    lead_position_1 = Point2D(-30, 0)
    lead_speed_1 = Point2D(4, 0)
    lead_size = 0.6
    lead_major_axis_1 = 0.6
    lead_minor_axis_1 = 0.2
    lead_orientation_1 = 0

    ego_vehicle_1 = Vehicle(idm_1, route_1, ego_position_1, ego_speed_1,
                            ego_size, ego_major_axis_1, ego_minor_axis_1, ego_orientation_1)
    lead_vehicle_1 = Vehicle(idm_1, route_1, lead_position_1, lead_speed_1,
                             lead_size, lead_major_axis_1, lead_minor_axis_1, lead_orientation_1)

    # initializations for ego vehicle 1
    ego_position_2 = Point2D(0, -50)
    ego_speed_2 = Point2D(0, 10)
    ego_size = 0.6
    ego_major_axis_2 = 0.6
    ego_minor_axis_2 = 0.2
    ego_orientation_2 = 90.0

    lead_position_2 = Point2D(0, -20)
    lead_speed_2 = Point2D(0, 4)
    lead_size = 0.6
    lead_major_axis_2 = 0.6
    lead_minor_axis_2 = 0.2
    lead_orientation_2 = 90.0

    ego_vehicle_2 = Vehicle(idm_2, route_2, ego_position_2, ego_speed_2,
                            ego_size, ego_major_axis_2, ego_minor_axis_2, ego_orientation_2)
    lead_vehicle_2 = Vehicle(idm_2, route_2, lead_position_2, lead_speed_2,
                             lead_size, lead_major_axis_2, lead_minor_axis_2, lead_orientation_2)

    return ego_vehicle_1, lead_vehicle_1, ego_vehicle_2, lead_vehicle_2


def lanes_plotter(
        ax,
):
    """
    function to visualize the lanes of the environment
    """
    # visualizing the road boundaries
    ax.plot(x_horizontal_lane, y_horizontal_lane, 'r--')
    ax.plot(x_vertical_lane, y_vertical_lane, 'r--')

    ax.plot(x_horizontal_lane, boundaries_left_lane, 'b-')
    ax.plot(x_horizontal_lane, boundaries_right_lane, 'b-')

    ax.plot(boundaries_left_lane, y_vertical_lane, 'g-')
    ax.plot(boundaries_right_lane, y_vertical_lane, 'g-')

    ax.axis('equal')

    return


if __name__ == "__main__":

    fig, ax = plt.subplots()
    ax.axis('equal')

    # time params for computing the future trajectory
    time_horizon = 50
    time_step = 0.1

    # step in real world
    time_move = 1
    sim_time = 10

    # get initial vehicle information
    ego_vehicle_1, lead_vehicle_1, ego_vehicle_2, lead_vehicle_2 = get_vehicle_info()

    for step in range(sim_time):
        ax.clear()

        # plot the environment
        lanes_plotter(ax)

        print("simulation time step:", step)

        ego_predictions_with_circular_noise_1, lead_predictions_with_circular_noise_1 = circular_predictions(
            ego_vehicle_1, lead_vehicle_1, time_horizon, time_step)
        ego_predictions_with_circular_noise_2, lead_predictions_with_circular_noise_2 = circular_predictions(
            ego_vehicle_2, lead_vehicle_2, time_horizon, time_step)

        ego_predictions_with_elliptical_noise_1, lead_predictions_with_elliptical_noise_1 = elliptical_predictions(
            ego_vehicle_1, lead_vehicle_1, time_horizon, time_step)
        ego_predictions_with_elliptical_noise_2, lead_predictions_with_elliptical_noise_2 = elliptical_predictions(
            ego_vehicle_2, lead_vehicle_2, time_horizon, time_step)

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
            ego_vehicle_1.position.x, ego_vehicle_1.velocity.x, lead_vehicle_1.position.x, lead_vehicle_1.velocity.x)
        print("time to collision:", TTC_1, "seconds!")

        TTC_2 = time_to_collision(
            ego_vehicle_2.position.y, ego_vehicle_2.velocity.y, lead_vehicle_2.position.y, lead_vehicle_2.velocity.y)
        print("time to collision:", TTC_2, "seconds!")

        # take a step in the real world
        ego_vehicle_1.position.x += ego_vehicle_1.velocity.x * time_move
        lead_vehicle_1.position.x += lead_vehicle_1.velocity.x * time_move

        ego_vehicle_2.position.y += ego_vehicle_2.velocity.y * time_move
        lead_vehicle_2.position.y += lead_vehicle_2.velocity.y * time_move

        print("-------------------------------------")

    plt.show()
