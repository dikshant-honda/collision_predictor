import math
import numpy as np
import matplotlib.pyplot as plt

class IDM:
    def __init__(self, desired_speed=25, time_headway=1.5, min_gap=2, max_acceleration=1.5, comfortable_deceleration=1,
                 delta=4, acceleration_exponent=1.5):
        self.desired_speed = desired_speed
        self.time_headway = time_headway
        self.min_gap = min_gap
        self.max_acceleration = max_acceleration
        self.comfortable_deceleration = comfortable_deceleration
        self.delta = delta
        self.acceleration_exponent = acceleration_exponent

    def calculate_acceleration(self, ego_vehicle_speed, lead_vehicle_speed, ego_vehicle_distance):
        desired_gap = self.min_gap + ego_vehicle_speed * self.time_headway + (
                (ego_vehicle_speed * (ego_vehicle_speed - lead_vehicle_speed)) / (
                2 * self.comfortable_deceleration * math.sqrt(self.max_acceleration)))

        acceleration = self.max_acceleration * (1 - math.pow(ego_vehicle_speed / self.desired_speed, self.delta) -
                                                math.pow(desired_gap / ego_vehicle_distance, 2))

        return acceleration

def time_to_collision(position1, velocity1, position2, velocity2):
    relative_position = np.array(position1) - np.array(position2)
    relative_velocity = np.array(velocity1) - np.array(velocity2)
    
    if np.dot(relative_position, relative_velocity) >= 0:
        return float("inf")
    
    time = -np.dot(relative_position, relative_velocity) / np.dot(relative_velocity, relative_velocity)
    if time < 0:
        return 0
    
    return time

def predict_trajectory(idm: IDM, initial_speed, lead_speed, gap, initial_position, time_horizon, time_step):
    speed = initial_speed
    position = initial_position
    trajectory = [(position, speed)]
    pos = [position]
    time = []

    for t in range(time_horizon):
        acceleration = idm.calculate_acceleration(speed, lead_speed, gap)  # Assuming no lead vehicle
        speed += acceleration * time_step
        position += speed * time_step
        trajectory.append((position, speed))
        pos.append(position)
        time.append(t*time_step)
    
    time.append(1.0)

    return trajectory, pos, time

def plot(x_pos, y_pos):
    line1.set_data(x_pos[0], y_pos)
    line2.set_data(x_pos[1], y_pos)

    # ax.plot(x_pos[0][0], y_pos[0], "*")
    # ax.plot(x_pos[1][0], y_pos[0], "o")

    ax.set_xlim(-1, 50)
    ax.set_ylim(-1, 1)

    # Redraw the plot
    plt.draw()
    plt.pause(1)

if __name__ == '__main__':
    ego_pos_data = []
    lead_pos_data = []
    time_data = []

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Create an empty plot
    line1, = ax.plot([], [], 'r-')
    line2, = ax.plot([], [], 'b*')

    idm = IDM()
    initial_speed = 20  # Initial speed of the vehicle
    initial_position = 0  # Initial position of the vehicle
    time_horizon = 10  # Time horizon for trajectory prediction (in seconds)
    time_step = 0.1  # Time step for trajectory prediction (in seconds)

    # trajectory, pos, _ = predict_trajectory(idm, initial_speed, 0, math.inf, initial_position, time_horizon, time_step)

    ego_vehicle = IDM()
    ego_vehicle_speed = 20
    ego_initial_position = 0

    lead_vehicle = IDM()
    lead_vehicle_speed = 5
    lead_initial_position = 10

    # initial iteration
    init_gap = lead_initial_position - ego_initial_position
    ego_trajectory, ego_pos, time = predict_trajectory(ego_vehicle ,ego_vehicle_speed, lead_vehicle_speed, init_gap, ego_initial_position, time_horizon, time_step)
    lead_trajectory, lead_pos, time = predict_trajectory(lead_vehicle, lead_vehicle_speed, 0, math.inf, lead_initial_position, time_horizon, time_step)

    for t in range(10):
        gap = lead_trajectory[1][0] - ego_trajectory[1][0]
        ego_trajectory, ego_pos, time = predict_trajectory(ego_vehicle ,ego_vehicle_speed, lead_vehicle_speed, init_gap, ego_pos[1], time_horizon, time_step)
        lead_trajectory, lead_pos, time = predict_trajectory(lead_vehicle, lead_vehicle_speed, 0, math.inf, lead_pos[1], time_horizon, time_step)
        
        ego_pos_data.clear()
        lead_pos_data.clear()

        data = [ego_pos, lead_pos]
        # for t, (position, speed) in enumerate(ego_trajectory):
        #     print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")

        # for t, (position, speed) in enumerate(lead_trajectory):
        #     print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")

        # print("-------------------------------------------------")

        time = time_to_collision(ego_trajectory[0], ego_trajectory[1], lead_trajectory[0], lead_trajectory[1])
        print(time)
        plot(data, np.zeros((len(ego_pos))))

        # plot(ego_pos, np.zeros((len(ego_pos))))
        # plot(lead_pos, np.zeros((len(lead_pos))))

    plt.show()
