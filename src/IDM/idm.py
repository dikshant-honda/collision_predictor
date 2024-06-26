#! /usr/bin/env python3

import math

import matplotlib.pyplot as plt
import numpy as np
from numpy import ndarray

from IDM.frenet import *
from IDM.path import Path


class IDM:
    def __init__(
            self,
            desired_speed: float = 25,
            time_headway: float = 1.5,
            min_gap: float = 2,
            max_acceleration: float = 1.5,
            comfortable_deceleration: float = 2,
            delta: int = 4,
    ) -> None:
        """
        Intelligent Driver Model (IDM) class for calculating the acceleration given
        the dynamics of the environment

        args:
            desired_speed: desired speed of vehicle "n"
            time_headway: safety time gap of vehicle "n"
            min_gap: jam distance of the vehicle "n"
            max_acceleration: maximum acceleration of the vehicle "n"
            comfortable_deceleration: desired deceleration of the vehicle "n" 
            delta: exponent for current speed / desired speed
        """
        self.desired_speed = desired_speed
        self.time_headway = time_headway
        self.min_gap = min_gap
        self.max_acceleration = max_acceleration
        self.comfortable_deceleration = comfortable_deceleration
        self.delta = delta

    def calculate_acceleration(
            self,
            ego_vehicle_speed: ndarray[np.float64],
            lead_vehicle_speed: ndarray[np.float64],
            ego_vehicle_distance: float,
    ) -> float:
        """
        Function to calculate the accleration after IDM calculations

        args:
            ego_vehicle_speed: current speed of the ego vehicle
            lead_vehicle_speed: speed of the vehicle ahead of ego vehicle
            ego_vehicle_distance: distance between ego vehicle and lead vehicle,
                                  infinity, if there's no vehicle ahead of ego
        """
        desired_gap = self.min_gap + ego_vehicle_speed * self.time_headway + (
            (ego_vehicle_speed * (ego_vehicle_speed - lead_vehicle_speed)) / (
                2 * math.sqrt(self.max_acceleration * self.comfortable_deceleration)))

        acceleration = self.max_acceleration * (1 - math.pow(ego_vehicle_speed / self.desired_speed, self.delta) -
                                                math.pow(desired_gap / ego_vehicle_distance, 2))

        return max(-self.comfortable_deceleration, acceleration)


def predict_trajectory(
        idm: IDM,
        ego_position: Point2D,
        ego_velocity: Point2D,
        lead_position: Point2D,
        lead_velocity: Point2D,
        path: list,
        time_horizon: int,
        time_step: int,
        interpolate_back_path: int = 100,
) -> list:
    """
    Function to compute the future trajecory of the vehicle using IDM analysis

    Returns the future trajectory coordinates of the ego and the lead 
    vehicle in a list at every time step

    args:
        idm: defining an IDM instance for calculating the accelaration value
        ego_position: current positon of the ego vehicle
        ego_velocity: current speed of the ego vehicle
        lead_position: current positon of the lead vehicle
        lead_velocity: current speed of the lead vehicle 
        path: center lane coordinates of the lane on which vehicle is currently located, type list(Point2D)
        time_horizon: duration over which you want to predict the trajectory
        time_step: discrete interval at which you update the state variables of the system during the trajectory prediction 
        interpolate_back_path: interpolate back to path after this number of steps
    """

    ego_trajectory = []
    lead_trajectory = []
    time = []

    curr_lead_x, curr_lead_y = lead_position.x, lead_position.y

    s_map = get_s_map(path)

    ego_position_in_frenet = get_frenet(ego_position, path, s_map)
    lead_position_in_frenet = get_frenet(lead_position, path, s_map)

    for t in range(time_horizon):
        # compute the gap between the ego and lead vehicle
        gap = ego_position_in_frenet.s - lead_position_in_frenet.s

        # compute speed
        ego_speed = np.sqrt(ego_velocity.x ** 2 + ego_velocity.y ** 2)
        lead_speed = np.sqrt(lead_velocity.x ** 2 + lead_velocity.y ** 2)

        # compute IDM accleration based on this dynamics
        acceleration = idm.calculate_acceleration(ego_speed, lead_speed, gap)

        # update the dynamics of the ego vehicle
        ego_speed += acceleration * time_step
        ego_position_in_frenet.s += ego_speed * time_step
        time_steps = t * time_step

        # tracking back towards the center line linearly
        ego_position_in_frenet.d -= t * ego_position_in_frenet.d / interpolate_back_path

        ego_position = get_xy(ego_position_in_frenet, path, s_map)

        ego_trajectory.append(ego_position)
        time.append(time_steps)

        # update the dynamics of the lead vehicle
        # assuming the lead vehicle is moving with a constant velocity and its offset is zero from the center lane
        lead_position_in_frenet.s += lead_speed * time_step
        lead_position_in_frenet.d = 0

        curr_lead_x += lead_velocity.x * time_step
        curr_lead_y += lead_velocity.y * time_step

        lead_trajectory.append(Point2D(curr_lead_x, curr_lead_y))

    return [time, ego_trajectory, lead_trajectory]


def position_plot(
        ego_trajectory: list,
        lead_trajectory: list,
) -> None:
    """
    Function to plot the future ego and lead trajectory positions

    args:
        ego_trajectory: future ego trajectory Point2D(x, y)
        lead_trajectory: future lead trajectory Point2D(x, y)
    """
    ax_position.clear()

    # visualization parameters
    ax_position.set_xlabel("x(m)")
    ax_position.set_ylabel("y(m)")
    ax_position.set_xlim(-1, 150)
    ax_position.set_ylim(-1, 1)

    x_ego, y_ego = route.points_to_coordinates(ego_trajectory)
    x_lead, y_lead = route.points_to_coordinates(lead_trajectory)

    # visualizing the roads
    ax_position.plot(x_coords, y_coords, 'y--')
    ax_position.plot(x_coords, boundaries_left, "g")
    ax_position.plot(x_coords, boundaries_right, "g")

    ax_position.plot(x_ego[0], y_ego[0], "o")
    ax_position.plot(x_lead[0], y_lead[0], "o")

    line1, = ax_position.plot([], [], 'r-', label="ego trajectory")
    line2, = ax_position.plot([], [], 'b--', label="lead trajectory")
    line1.set_data(x_ego, y_ego)
    line2.set_data(x_lead, y_lead)

    # visualize
    plt.legend()
    plt.draw()
    plt.pause(1.0)

    return


def time_plot(
        ax_time,
        time: float,
        ego_trajectory: list,
        lead_trajectory: list,
) -> None:
    """
    Function to plot the future ego and lead trajectory positions with respect to time

    args:
        ax_time: matplotlib tool for plotting
        time: time step per future predictions
        ego_trajectory: future ego trajectory Point2D(x, y)
        lead_trajectory: future lead trajectory Point2D(x, y)
    """
    ax_time.clear()

    # visualization parameters
    ax_time.set_xlabel("time(s)")
    ax_time.set_ylabel("position(m)")
    ax_time.set_xlim(0, 10)
    ax_time.set_ylim(0, 100)

    x_ego, y_ego = route.points_to_coordinates(ego_trajectory)
    x_lead, y_lead = route.points_to_coordinates(lead_trajectory)

    ax_time.plot(time[0], x_ego[0], "o")
    ax_time.plot(time[0], x_lead[0], "o")

    line1, = ax_time.plot([], [], 'r-', label="ego trajectory")
    line2, = ax_time.plot([], [], 'b--', label="lead trajectory")
    line1.set_data(time, x_ego)
    line2.set_data(time, x_lead)

    # visualize
    plt.legend()
    plt.draw()
    plt.pause(1.0)

    return


def update(ego_position: Point2D, ego_velocity: Point2D, lead_position: Point2D, lead_velocity: Point2D, time=0.5):
    ego_position.x += ego_velocity.x * time
    lead_position.x += lead_velocity.x * time
    return ego_position, lead_position


if __name__ == "__main__":

    fig_position, ax_position = plt.subplots()
    # fig_time, ax_time = plt.subplots()

# """
# sample test code for getting IDM based future trajectory predictions:

    time_horizon = 50
    time_step = 0.1

    number_of_points = 100

    x_coords = np.linspace(0, 150, number_of_points)
    y_coords = np.linspace(0, 0, number_of_points)

    boundaries_left = np.linspace(0.5, 0.5, number_of_points)
    boundaries_right = np.linspace(-0.5, -0.5, number_of_points)

    # initializations
    ego_position = Point2D(0, 0)
    ego_speed = Point2D(15, 0)

    lead_position = Point2D(50, 0)
    lead_speed = Point2D(10, 0)

    # obtaining the path from the route
    route = Path(x_coords, y_coords)
    path = route.get_path()

    # calling the IDM class object
    idm = IDM()

    # running the check 10 times
    for _ in range(10):
        result = predict_trajectory(
            idm, ego_position, ego_speed, lead_position, lead_speed, path, time_horizon, time_step)

        time, ego_trajectory, lead_trajectory = result

        # dynamically plotting the positions
        position_plot(ego_trajectory, lead_trajectory)

        # dynamically plotting the position with respect to time
        # time_plot(ax_time, time, ego_trajectory, lead_trajectory)

        # take a step in the real world
        ego_position, lead_position = update(
            ego_position, ego_speed, lead_position, lead_speed)

    plt.show()

# """
