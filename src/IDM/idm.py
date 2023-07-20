import math

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import NDArray


class IDM:
    def __init__(
            self,
            desired_speed: float = 25,
            time_headway: float = 1.5,
            min_gap: float = 2,
            max_acceleration: float = 1.5,
            comfortable_deceleration: float = 1,
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
            ego_vehicle_speed: float,
            lead_vehicle_speed: float,
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

        return acceleration


def time_to_collision(
        position1: NDArray[np.float64],
        velocity1: float,
        position2: NDArray[np.float64],
        velocity2: float,
) -> float:
    """
    Function to compute the time to collision (TTC) between two traffic participants.

    args:
        position1, velocity1: position and velocity of the first vehicle
        position2, velocity2: position and velocity of the second vehicle
    """
    relative_position = np.array(position1) - np.array(position2)
    relative_velocity = np.array(velocity1) - np.array(velocity2)

    if np.dot(relative_position, relative_velocity) >= 0:
        return float("inf")

    time = -np.dot(relative_position, relative_velocity) / \
        np.dot(relative_velocity, relative_velocity)
    if time < 0:
        return 0

    return time


def predict_trajectory(
        idm: IDM,
        ego_position: float,
        ego_speed: float,
        lead_position: float,
        lead_speed: float,
        time_horizon: int,
        time_step: int,
) -> list:
    """
    Function to compute the future trajecory of the vehicle using IDM analysis

    Returns the future trajectory coordinates of the ego and the lead 
    vehicle in a list at every time step

    args:
        idm: defining an IDM instance for calculating the accelaration value
        ego_position: current positon of the ego vehicle
        ego_speed: current speed of the ego vehicle
        lead_position: current positon of the lead vehicle
        lead_speed: current speed of the lead vehicle 
        time_horizon: duration over which you want to predict the trajectory
        time_step: discrete interval at which you update the state variables of the system during the trajectory prediction 
    """

    ego_trajectory = []
    lead_trajectory = []
    time = []

    for t in range(time_horizon):
        # compute the gap between the ego and lead vehicle
        gap = lead_position - ego_position

        # compute IDM accleration based on this dynamics
        acceleration = idm.calculate_acceleration(ego_speed, lead_speed, gap)

        # update the dynamics of the ego vehicle
        ego_speed += acceleration * time_step
        ego_position += ego_speed * time_step
        time_steps = t * time_step

        ego_trajectory.append(ego_position)
        time.append(time_steps)

        # update the dynamics of the traffic vehicle
        # assuming the lead vehicle is moving with a constant velocity
        lead_position = lead_position + lead_speed * time_step
        lead_trajectory.append(lead_position)

    return [time, ego_trajectory, lead_trajectory]


def position_plot(
        ego_trajectory: list,
        lead_trajectory: list,
) -> None:
    """
    Function to plot the future ego and lead trajectory positions

    args:
        ego_trajectory: future ego trajectory (x, y)
        lead_trajectory: future lead trajectory (x, y)
    """
    fig, ax = plt.subplots()
    ax.clear()

    # visualization parameters
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_xlim(-1, 150)
    ax.set_ylim(-1, 1)

    # consider movement in x-direction only for time being
    y = np.linspace(0, 0, time_horizon)

    ax.plot(ego_trajectory[0], y[0], "o")
    ax.plot(lead_trajectory[0], y[0], "o")

    line1, = ax.plot([], [], 'r-', label="ego trajectory")
    line2, = ax.plot([], [], 'b--', label="lead trajectory")
    line1.set_data(ego_trajectory, y)
    line2.set_data(lead_trajectory, y)

    # visualize
    plt.legend()
    plt.draw()
    plt.pause(0.1)


def time_plot(
        time: float,
        ego_trajectory: list,
        lead_trajectory: list,
) -> None:
    """
    Function to plot the future ego and lead trajectory positions with respect to time

    args:
        time: time step per future predictions
        ego_trajectory: future ego trajectory (x, y)
        lead_trajectory: future lead trajectory (x, y)
    """
    fig, ax = plt.subplots()
    ax.clear()

    # visualization parameters
    ax.set_xlabel("time(s)")
    ax.set_ylabel("position(m)")
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 100)

    ax.plot(time[0], ego_trajectory[0], "o")
    ax.plot(time[0], lead_trajectory[0], "o")

    line1, = ax.plot([], [], 'r-', label="ego trajectory")
    line2, = ax.plot([], [], 'b--', label="lead trajectory")
    line1.set_data(time, ego_trajectory)
    line2.set_data(time, lead_trajectory)

    # visualize
    plt.legend()
    plt.draw()
    plt.pause(0.1)

def update(ego_position, ego_velocity, lead_position, lead_velocity, time=0.5):
    ego_position += ego_velocity * time
    lead_position += lead_velocity * time
    return ego_position, lead_position

if __name__ == "__main__":
    ego_position = 0
    ego_speed = 15

    lead_position = 50
    lead_speed = 10

    time_horizon = 50
    time_step = 0.1

    idm = IDM()

    for _ in range(10):
        result = predict_trajectory(
            idm, ego_position, ego_speed, lead_position, lead_speed, time_horizon, time_step)

        time, ego_trajectory, lead_trajectory = result

        position_plot(ego_trajectory, lead_trajectory)

        time_plot(time, ego_trajectory, lead_trajectory)

        ego_position, lead_position = update(ego_position, ego_speed, lead_position, lead_speed)

    plt.show()