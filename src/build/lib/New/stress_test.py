#! /usr/bin/env python3

import argparse
import time

import matplotlib.pyplot as plt
import numpy as np

from env_info.cross_intersection import *
from IDM.frenet import Point2D
from IDM.idm import IDM, predict_trajectory
from IDM.path import Path
from New.circular_noise import add_noise as add_circular_noise
from New.circular_overlap import overlap as circular_overlap
from New.circular_overlap import plotter as circle_plotter
from New.elliptical_noise import add_noise as add_elliptical_noise
from New.elliptical_overlap import overlap as elliptical_overlap
from New.elliptical_overlap import plotter as ellipse_plotter
from New.TTC import time_to_collision


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


def create_vehicle(
        x_position: float,
        y_position: float,
        x_speed: float,
        y_speed: float,
        route: Path,
        size: float,
        major_axis: float,
        minor_axis: float,
        orientation: float,
) -> Vehicle:
    """
    Function to create the vehicle object data type

    args:
        x_position: x position of the vehicle
        y_position: y position of the vehicle
        x_speed: speed of the vehicle in x direction
        y_speed: speed of the vehicle in y direction
        route: path which the vehicle will take
        size: size of the vehicle
        major_axis: uncertainity along car movement axis
        minor_axis: uncertainity along the car movement perpendicular axis
        orientation: orientation of the vehicle
    """

    idm = IDM()
    position = Point2D(x_position, y_position)
    velocity = Point2D(x_speed, y_speed)

    return Vehicle(idm, route, position, velocity, size, major_axis, minor_axis, orientation)


def get_vehicle_info():

    # obtaining the path from the route
    route_1 = Path(x_horizontal_lane, y_horizontal_lane, number_of_points)
    route_2 = Path(x_vertical_lane, y_vertical_lane, number_of_points)

    ego_vehicle_1 = create_vehicle(-100, 0, 10, 0, route_1, 0.6, 0.6, 0.2, 0)
    lead_vehicle_1 = create_vehicle(-30, 0, 4, 0, route_1, 0.6, 0.6, 0.2, 0)

    ego_vehicle_2 = create_vehicle(0, -100, 0, 10, route_2, 0.6, 0.6, 0.2, 90)
    lead_vehicle_2 = create_vehicle(0, -20, 0, 4, route_2, 0.6, 0.6, 0.2, 90)

    ego_vehicle_3 = create_vehicle(-50, 0, 5, 0, route_1, 0.6, 0.6, 0.2, 0)
    lead_vehicle_3 = create_vehicle(-10, 0, 4, 0, route_1, 0.6, 0.6, 0.2, 0)

    ego_vehicle_4 = create_vehicle(0, -70, 0, 7, route_2, 0.6, 0.6, 0.2, 90)
    lead_vehicle_4 = create_vehicle(0, -30, 0, 3, route_2, 0.6, 0.6, 0.2, 90)

    return ego_vehicle_1, lead_vehicle_1, ego_vehicle_2, lead_vehicle_2, ego_vehicle_3, lead_vehicle_3, ego_vehicle_4, lead_vehicle_4


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

    return vehicle


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
    ego_vehicle_1, lead_vehicle_1, ego_vehicle_2, lead_vehicle_2, ego_vehicle_3, lead_vehicle_3, ego_vehicle_4, lead_vehicle_4 = get_vehicle_info()

    # uncertainity type
    uncertainity = args.uncertainity_type

    # plotting tool
    to_plot = args.plot
    fig, ax = plt.subplots()
    ax.axis('equal')

    # evaluation time
    total_time = 0

    for step in range(sim_time):
        # prediction start time
        start_time = time.time()

        ax.clear()

        # plot the environment
        if to_plot:
            lanes_plotter(ax)

        print("simulation time step:", step)

        if uncertainity == "circular":
            ego_predictions_with_circular_noise_1, lead_predictions_with_circular_noise_1 = circular_predictions(
                ego_vehicle_1, lead_vehicle_1, time_horizon, time_step)
            ego_predictions_with_circular_noise_2, lead_predictions_with_circular_noise_2 = circular_predictions(
                ego_vehicle_2, lead_vehicle_2, time_horizon, time_step)

            # circular overlap check
            for time_ in range(time_horizon):
                overlap_area_1 = circular_overlap(
                    ego_predictions_with_circular_noise_1[time_], lead_predictions_with_circular_noise_1[time_])
                overlap_area_2 = circular_overlap(
                    ego_predictions_with_circular_noise_2[time_], lead_predictions_with_circular_noise_2[time_])

                if to_plot:
                    circle_plotter(
                        ax, ego_predictions_with_circular_noise_1[time_], lead_predictions_with_circular_noise_1[time_])
                    circle_plotter(
                        ax, ego_predictions_with_circular_noise_2[time_], lead_predictions_with_circular_noise_2[time_])

                if overlap_area_1 > 0.1:
                    print("collision probability for ego 1:", overlap_area_1,
                          "after:", time_*time_step, "seconds!")

                if overlap_area_2 > 0.1:
                    print("collision probability for ego 2:", overlap_area_2,
                          "after:", time_*time_step, "seconds!")

        if uncertainity == "elliptical":
            ego_predictions_with_elliptical_noise_1, lead_predictions_with_elliptical_noise_1 = elliptical_predictions(
                ego_vehicle_1, lead_vehicle_1, time_horizon, time_step)
            ego_predictions_with_elliptical_noise_2, lead_predictions_with_elliptical_noise_2 = elliptical_predictions(
                ego_vehicle_2, lead_vehicle_2, time_horizon, time_step)
            ego_predictions_with_elliptical_noise_3, lead_predictions_with_elliptical_noise_3 = elliptical_predictions(
                ego_vehicle_3, lead_vehicle_3, time_horizon, time_step)
            ego_predictions_with_elliptical_noise_4, lead_predictions_with_elliptical_noise_4 = elliptical_predictions(
                ego_vehicle_4, lead_vehicle_4, time_horizon, time_step)

            # elliptical overlap check
            for time_ in range(time_horizon):
                overlap_area_1 = elliptical_overlap(
                    ego_predictions_with_elliptical_noise_1[time_], lead_predictions_with_elliptical_noise_1[time_])
                overlap_area_2 = elliptical_overlap(
                    ego_predictions_with_elliptical_noise_2[time_], lead_predictions_with_elliptical_noise_2[time_])
                overlap_area_3 = elliptical_overlap(
                    ego_predictions_with_elliptical_noise_3[time_], lead_predictions_with_elliptical_noise_3[time_])
                overlap_area_4 = elliptical_overlap(
                    ego_predictions_with_elliptical_noise_4[time_], lead_predictions_with_elliptical_noise_4[time_])


                if to_plot:
                    ellipse_plotter(
                        ax, ego_predictions_with_elliptical_noise_1[time_], lead_predictions_with_elliptical_noise_1[time_])
                    ellipse_plotter(
                        ax, ego_predictions_with_elliptical_noise_2[time_], lead_predictions_with_elliptical_noise_2[time_])

                if overlap_area_1 > 0.1:
                    print("collision probability for ego 1:", overlap_area_1,
                          "after:", time_*time_step, "seconds!")

                if overlap_area_2 > 0.1:
                    print("collision probability for ego 2:", overlap_area_2,
                          "after:", time_*time_step, "seconds!")
                    
                if overlap_area_3 > 0.1:
                    print("collision probability for ego 3:", overlap_area_3,
                          "after:", time_*time_step, "seconds!")
                    
                if overlap_area_4 > 0.1:
                    print("collision probability for ego 4:", overlap_area_4,
                          "after:", time_*time_step, "seconds!")

        # time to collision evaluation
        # TTC_1 = time_to_collision(
        #     ego_vehicle_1.position, ego_vehicle_1.velocity, lead_vehicle_1.position, lead_vehicle_1.velocity)
        # print("time to collision for ego 1:", TTC_1, "seconds!")

        # TTC_2 = time_to_collision(
        #     ego_vehicle_2.position, ego_vehicle_2.velocity, lead_vehicle_2.position, lead_vehicle_2.velocity)
        # print("time to collision for ego 2:", TTC_2, "seconds!")

        # prediction end time
        end_time = time.time()

        prediction_time = end_time - start_time

        print("prediction time:", prediction_time)

        total_time += prediction_time

        # take a step in the real world
        ego_vehicle_1 = update(ego_vehicle_1, time_move)
        lead_vehicle_1 = update(lead_vehicle_1, time_move)
        ego_vehicle_2 = update(ego_vehicle_2, time_move)
        lead_vehicle_2 = update(lead_vehicle_2, time_move)
        ego_vehicle_3 = update(ego_vehicle_3, time_move)
        lead_vehicle_3 = update(lead_vehicle_3, time_move)
        ego_vehicle_4 = update(ego_vehicle_4, time_move)
        lead_vehicle_4 = update(lead_vehicle_4, time_move)

        print("-------------------------------------")

    print("average computation time:", total_time / sim_time)

    if to_plot:
        plt.show()
