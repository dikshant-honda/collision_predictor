import numpy as np
import matplotlib.pyplot as plt

u = 3                                           # initial velocity considered to be 1 m/s    
a = 0                                           # considering acceleration 0 for time being
vel_profile = []                                # velocity profile
horizon = 10                                    # length of velocity profile 
future_horizon = 20                             # number of time steps
x_car = [0]*future_horizon                      # current x coordinate of car
max_time = 5                                    # future time horizon
t = np.linspace(0, max_time,future_horizon)     # time steps
obs_distance = 40                               # distance from the obstacle

# for car

# for pedestrian
y_ped = [-20]*future_horizon
x_ped = [20]*future_horizon
u_ped = 0.1

# plotting 
y_car = [0]*future_horizon 
plt.title("future trajectory predictor")
plt.xlabel("x")
plt.ylabel("y")
plt.xlim(-5,30)
plt.ylim(-25,10)

# stopping function
def stop(car,ped):
    if np.sqrt(car**2+ped**2) < 0.5:
        print("!!STOP!!")
        return False
    return True

### for ego agent
# while x_car[-1] < obs_distance:
#     for _ in range(horizon):
#         noise = np.random.random()              # adding noise to the velocity profile
#         vel_profile.append(u+noise)

#     avg_vel = np.mean(vel_profile)

#     x_new = x_car[1] + np.multiply(avg_vel, t) + 0.5*a*np.square(t)

#     x_car = x_new
#     print(x_car)
#     plt.plot(x_car, y_car)
#     plt.plot(x_car[1], y_car[1], marker='>', markersize=7)
#     plt.pause(0.2)

### for pedestrian
# while y_ped[-1] < 0:
#     for _ in range(horizon):
#         # noise = np.random.random()              # adding noise to the velocity profile
#         noise = 0
#         vel_profile.append(u_ped+noise)

#     avg_vel = np.mean(vel_profile)

#     y_new = y_ped[1] + np.multiply(avg_vel, t) + 0.5*a*np.square(t)

#     y_ped = y_new

#     plt.plot(x_ped, y_ped)
#     plt.plot(x_ped[1], y_ped[1], marker='^', markersize=7)
#     plt.pause(0.2)

#     print(y_ped)

### MERGE
while stop(y_car[-1], y_ped[-1]):

    # car
    for _ in range(horizon):
        noise = np.random.random()              # adding noise to the velocity profile
        vel_profile.append(u+noise)

    avg_vel = np.mean(vel_profile)

    x_new = x_car[1] + np.multiply(avg_vel, t) + 0.5*a*np.square(t)
    x_car = x_new

    # pedestrian
    for _ in range(horizon):
        # noise = np.random.random()              # adding noise to the velocity profile
        noise = 0
        vel_profile.append(u_ped+noise)

    avg_vel = np.mean(vel_profile)

    y_new = y_ped[1] + np.multiply(avg_vel, t) + 0.5*a*np.square(t)
    y_ped = y_new

    # plotting
    plt.plot(x_car, y_car)
    plt.plot(x_car[1], y_car[1], marker='>', markersize=7)
    plt.plot(x_ped, y_ped)
    plt.plot(x_ped[1], y_ped[1], marker='^', markersize=7)
    print(x_car[-1])
    plt.pause(0.2)

plt.show()