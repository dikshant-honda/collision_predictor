import math
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


def predict_trajectory(idm, initial_speed, lead_speed, gap, initial_position, time_horizon, time_step):
    speed = initial_speed
    position = initial_position
    trajectory = [(position, speed)]

    for _ in range(time_horizon):
        acceleration = idm.calculate_acceleration(speed, lead_speed, gap)  # Assuming no lead vehicle
        speed += acceleration * time_step
        position += speed * time_step
        trajectory.append((position, speed))

    return trajectory


idm = IDM()
initial_speed = 20  # Initial speed of the vehicle
initial_position = 0  # Initial position of the vehicle
time_horizon = 10  # Time horizon for trajectory prediction (in seconds)
time_step = 0.1  # Time step for trajectory prediction (in seconds)

trajectory = predict_trajectory(idm, initial_speed, 0, math.inf, initial_position, time_horizon, time_step)

ego_vehicle = IDM()
ego_vehicle_speed = 20
ego_initial_position = 0

lead_vehicle = IDM()
lead_vehicle_speed = 30
lead_initial_position = 10

# initial iteration
init_gap = lead_initial_position - ego_initial_position
ego_trajectory = predict_trajectory(ego_vehicle ,ego_vehicle_speed, lead_vehicle_speed, init_gap, ego_initial_position, time_horizon, time_step)
lead_trajectory = predict_trajectory(lead_vehicle, lead_vehicle_speed, 0, math.inf, lead_initial_position, time_horizon, time_step)

print("Predicted Trajectory:")
for t, (position, speed) in enumerate(ego_trajectory):
    print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")

for t, (position, speed) in enumerate(lead_trajectory):
    print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")

for t in range(10):
    gap = lead_trajectory[-1][0] - ego_trajectory[-1][0]
    ego_trajectory = predict_trajectory(ego_vehicle ,ego_vehicle_speed, lead_vehicle_speed, init_gap, ego_initial_position, time_horizon, time_step)
    lead_trajectory = predict_trajectory(lead_vehicle, lead_vehicle_speed, 0, math.inf, lead_initial_position, time_horizon, time_step)
    
    plt.plot(t, ego_trajectory[-1][0], "*")
    plt.plot(t, lead_trajectory[-1][0], "-")
    plt.pause(0.0001)

plt.show()