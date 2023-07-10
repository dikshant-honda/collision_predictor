import math

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


def predict_trajectory(idm, initial_speed, initial_position, time_horizon, time_step):
    speed = initial_speed
    position = initial_position
    trajectory = [(position, speed)]

    for t in range(time_horizon):
        acceleration = idm.calculate_acceleration(speed, 0, math.inf)  # Assuming no lead vehicle
        speed += acceleration * time_step
        position += speed * time_step
        trajectory.append((position, speed))

    return trajectory


# Example usage
idm = IDM()
initial_speed = 20  # Initial speed of the vehicle
initial_position = 0  # Initial position of the vehicle
time_horizon = 12  # Time horizon for trajectory prediction (in seconds)
time_step = 0.1  # Time step for trajectory prediction (in seconds)

trajectory = predict_trajectory(idm, initial_speed, initial_position, time_horizon, time_step)

print("Predicted Trajectory:")
for t, (position, speed) in enumerate(trajectory):
    print(f"Time: {t * time_step:.1f}s, Position: {position:.2f}m, Speed: {speed:.2f}m/s")
