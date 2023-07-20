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
        initial_position: NDArray[np.float64],
        initial_speed: float,
        lead_speed: float,
        gap: float,
        time_horizon: int,
        time_step: int,
) -> list:
    """
    Function to compute the future trajecory of the vehicle using IDM analysis

    Returns the future trajectory coordinates in a list at every time step

    args:
        idm: defining an IDM instance for calculating the accelaration value
        initial_position: current positon of the vehicle
        initial_speed: current speed of the vehicle
        lead_speed: speed of the vehicle ahead of it
        gap: distance between the current vehicle and the lead vehicle
        time_horizon: duration over which you want to predict the trajectory
        time_step: discrete interval at which you update the state variables of the system during the trajectory prediction 
    """
    speed = initial_speed
    position = initial_position
    trajectory = [(position, speed)]
    pos = [position]
    time = []

    for t in range(time_horizon):
        acceleration = idm.calculate_acceleration(
            speed, lead_speed, gap)  # Assuming no lead vehicle
        speed += acceleration * time_step
        position += speed * time_step
        trajectory.append((position, speed))
        pos.append(position)
        time.append(t*time_step)

    time.append(1.0)

    return trajectory, pos, time


# def plot(x_pos, y_pos):
#     line1.set_data(x_pos[0], y_pos)
#     line2.set_data(x_pos[1], y_pos)

#     # ax.plot(x_pos[0][0], y_pos[0], "*")
#     # ax.plot(x_pos[1][0], y_pos[0], "o")

#     ax.set_xlim(-1, 50)
#     ax.set_ylim(-1, 1)

#     # Redraw the plot
#     plt.draw()
#     plt.pause(1)


# if __name__ == '__main__':
#     ego_pos_data = []
#     lead_pos_data = []
#     time_data = []

#     # Create a figure and axis
#     fig, ax = plt.subplots()

#     # Create an empty plot
#     line1, = ax.plot([], [], 'r-')
#     line2, = ax.plot([], [], 'b*')

#     idm = IDM()
#     initial_speed = 20  # Initial speed of the vehicle
#     initial_position = 0  # Initial position of the vehicle
#     time_horizon = 10  # Time horizon for trajectory prediction (in seconds)
#     time_step = 0.1  # Time step for trajectory prediction (in seconds)

#     # trajectory, pos, _ = predict_trajectory(idm, initial_speed, 0, math.inf, initial_position, time_horizon, time_step)

#     ego_vehicle = IDM()
#     ego_vehicle_speed = 15
#     ego_initial_position = 0

#     lead_vehicle = IDM()
#     lead_vehicle_speed = 5
#     lead_initial_position = 10

#     # initial iteration
#     init_gap = lead_initial_position - ego_initial_position
#     ego_trajectory, ego_pos, time = predict_trajectory(
#         ego_vehicle, ego_initial_position, ego_vehicle_speed, lead_vehicle_speed, init_gap, time_horizon, time_step)
#     lead_trajectory, lead_pos, time = predict_trajectory(
#         lead_vehicle, lead_initial_position, lead_vehicle_speed, 0, math.inf, time_horizon, time_step)

#     for t in range(10):
#         gap = lead_trajectory[1][0] - ego_trajectory[1][0]
#         ego_trajectory, ego_pos, time = predict_trajectory(
#             ego_vehicle, ego_pos[1], ego_vehicle_speed, lead_vehicle_speed, init_gap, time_horizon, time_step)
#         lead_trajectory, lead_pos, time = predict_trajectory(
#             lead_vehicle, lead_pos[1], lead_vehicle_speed, 0, math.inf, time_horizon, time_step)

#         ego_pos_data.clear()
#         lead_pos_data.clear()

#         data = [ego_pos, lead_pos]
#         # for t, (position, speed) in enumerate(ego_trajectory):
#         #     print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")

#         # for t, (position, speed) in enumerate(lead_trajectory):
#         #     print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")

#         # print("-------------------------------------------------")

#         time = time_to_collision(
#             ego_trajectory[0], ego_trajectory[1], lead_trajectory[0], lead_trajectory[1])
#         print("time to collision:", time)

#         print("-------------------------------------")
#         plot(data, np.zeros((len(ego_pos))))

#         # plot(ego_pos, np.zeros((len(ego_pos))))
#         # plot(lead_pos, np.zeros((len(lead_pos))))

#     plt.show()

if __name__ == "__main__":
    idm = IDM()

    lead_speed = 20.0
    time_steps = 100
    time_step = 0.1

    v = [15]
    s = [10.0]

    print("initial speed:", v[0])

    for t in range(1, time_steps):

        acc = idm.calculate_acceleration(v[t-1], lead_speed, s[t-1])

        v.append(v[t-1] + acc * time_step)
        s.append(s[t-1] + v[t] * time_step)


    print("final speed:", v[-1])
    print("final spacing:", s[-1])

    print(v)
    print(s)