#! /usr/bin/env python3

import argparse
import time

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
    idm = IDM()

    # obtaining the path from the route
    route = Path(x_turning, y_turning, number_of_points)

    # initializations for ego vehicle 1
    ego_position = Point2D(-40, 0)
    ego_speed = Point2D(10, 0)
    ego_size = 0.6
    ego_major_axis = 0.6
    ego_minor_axis = 0.2
    ego_orientation = 0.0

    lead_position = Point2D(0, 5)
    lead_speed = Point2D(0, 4)
    lead_size = 0.6
    lead_major_axis = 0.6
    lead_minor_axis = 0.2
    lead_orientation = 0

    ego_vehicle = Vehicle(idm, route, ego_position, ego_speed,
                            ego_size, ego_major_axis, ego_minor_axis, ego_orientation)
    lead_vehicle = Vehicle(idm, route, lead_position, lead_speed,
                             lead_size, lead_major_axis, lead_minor_axis, lead_orientation)

    return ego_vehicle, lead_vehicle


def update(
        vehicle: Vehicle,
        time_move: float,
):
    """
    update the vehicle position in the real world

    args:
        vehicle: vehicle whose position need to be updated
        time_move: update time step
    """
    vehicle.position.x += vehicle.velocity.x * time_move
    vehicle.position.y += vehicle.velocity.y * time_move

    if vehicle.position.x == 0.0 and vehicle.position.y == 0.0:
        vehicle.velocity.y = vehicle.velocity.x
        vehicle.velocity.x = 0

    return vehicle


def lanes_plotter(
        ax,
):
    """
    function to visualize the lanes of the environment
    """
    # visualizing the road boundaries
    ax.plot(x_horizontal_lane, y_horizontal_lane, 'b--')
    ax.plot(x_vertical_lane, y_vertical_lane, 'g--')

    ax.plot(x_horizontal_lane, boundaries_left_lane, 'y-')
    ax.plot(x_horizontal_lane, boundaries_right_lane, 'y-')

    ax.plot(boundaries_left_lane, y_vertical_lane, 'y-')
    ax.plot(boundaries_right_lane, y_vertical_lane, 'y-')

    return


if __name__ == "__main__":
    # arguments parsing
    parser = argparse.ArgumentParser(
        description="future trajectory prediction tweaks")
    parser.add_argument("--time_horizon", type=int, default=50,
                        help="duration over which you want to predict the trajectory")
    parser.add_argument("--time_step", type=float, default=0.1,
                        help="discrete interval at which you update the state variables of the system during the trajectory prediction ")
    parser.add_argument("--uncertainity_type", type=str, default="elliptical",
                        help="type of uncertainity (circular or elliptical) in the future positions, usage: --circular or --elliptical")
    parser.add_argument("--plot", type=bool, default=False,
                        help="visualization tool")
    args = parser.parse_args()

    # time params for computing the future trajectory
    time_horizon = args.time_horizon
    time_step = args.time_step

    # step in real world
    time_move = 1
    sim_time = 10

    # get initial vehicle information
    ego_vehicle, lead_vehicle = get_vehicle_info()

    # uncertainity type
    uncertainity = args.uncertainity_type

    # plotting tool
    to_plot = args.plot
    fig, ax = plt.subplots()
    ax.axis('equal')

    for step in range(sim_time):
        # prediction start time
        start_time = time.time()

        ax.clear()

        # plot the environment
        if to_plot:
            lanes_plotter(ax)

        print("simulation time step:", step)

        if uncertainity == "circular":
            ego_predictions_with_circular_noise, lead_predictions_with_circular_noise = circular_predictions(
                ego_vehicle, lead_vehicle, time_horizon, time_step)

            # circular overlap check
            for time_ in range(time_horizon):
                overlap_area = circular_overlap(
                    ego_predictions_with_circular_noise[time_], lead_predictions_with_circular_noise[time_])
                
                if to_plot:
                    circle_plotter(
                        ax, ego_predictions_with_circular_noise[time_], lead_predictions_with_circular_noise[time_])
                   
                if overlap_area > 0.1:
                    print("collision probability for ego 1:", overlap_area,
                          "after:", time_*time_step, "seconds!")

               
        if uncertainity == "elliptical":
            ego_predictions_with_elliptical_noise, lead_predictions_with_elliptical_noise = elliptical_predictions(
                ego_vehicle, lead_vehicle, time_horizon, time_step)
            
            # elliptical overlap check
            for time_ in range(time_horizon):
                overlap_area = elliptical_overlap(
                    ego_predictions_with_elliptical_noise[time_], lead_predictions_with_elliptical_noise[time_])
              
                if to_plot:
                    ellipse_plotter(
                        ax, ego_predictions_with_elliptical_noise[time_], lead_predictions_with_elliptical_noise[time_])
               
                if overlap_area > 0.1:
                    print("collision probability for ego 1:", overlap_area,
                          "after:", time_*time_step, "seconds!")

                
        # time to collision evaluation
        TTC = time_to_collision(
            ego_vehicle.position, ego_vehicle.velocity, lead_vehicle.position, lead_vehicle.velocity)
        print("time to collision for ego:", TTC, "seconds!")


        # prediction end time
        end_time = time.time()

        prediction_time = end_time - start_time

        print("prediction time:", prediction_time)

        # take a step in the real world
        ego_vehicle = update(ego_vehicle, time_move)
        lead_vehicle = update(lead_vehicle, time_move)

        print("-------------------------------------")

    if to_plot:
        plt.show()
