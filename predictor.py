import numpy as np
import matplotlib.pyplot as plt
   
horizon = 10                                    # length of velocity profile 
future_horizon = 20                             # number of time steps
max_time = 5                                    # future time horizon
t = np.linspace(0, max_time,future_horizon)     # time steps

# for car
start_pos_car = 0
x_car = [start_pos_car]*future_horizon
u_car = 3                                       # initial velocity of the car    
a_car = 0                                       # acceleration of the car

# for pedestrian
start_pos_ped = -5
y_ped = [start_pos_ped]*future_horizon
u_ped = 0.5                                     # initial velocity of the pedestrian
a_ped = 0                                       # acceleration of the pedestrian

# plotting 
y_car = [0]*future_horizon 
x_ped = [20]*future_horizon
plt.title("Collision Predictor")
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

### MERGE
while stop(y_car[-1], y_ped[-1]):
    vel_profile_car = []
    vel_profile_ped = []
    for _ in range(horizon):
        noise_car = np.random.random()          # moise in the velocity profile of car
        vel_profile_car.append(u_car+noise_car)
        noise_ped = np.random.random()          # noise in the velocity profile of pedestrian
        vel_profile_ped.append(u_ped+noise_ped/10)

    # car dynamics
    avg_vel_car = np.mean(vel_profile_car)
    # update the x_car[1] by the actual position of the car
    x_new = x_car[1] + np.multiply(avg_vel_car, t) + 0.5*a_car*np.square(t)
    x_car = x_new

    # pedestrian dynamics
    avg_vel_ped = np.mean(vel_profile_ped)
    # update the y_ped[1] by the actual position of the pedestrian
    y_new = y_ped[1] + np.multiply(avg_vel_ped, t) + 0.5*a_ped*np.square(t)
    y_ped = y_new

    # plotting
    plt.plot(x_car, y_car)
    plt.plot(x_car[1], y_car[1], marker='>', markersize=7)
    plt.plot(x_ped, y_ped)
    plt.plot(x_ped[1], y_ped[1], marker='^', markersize=7)
    plt.pause(0.2)

plt.show()

'''
old code
### vars
vel_profile = []                                # velocity profile
obs_distance = 40                               # distance from the obstacle 

### for ego agent
while x_car[-1] < obs_distance:
    for _ in range(horizon):
        noise = np.random.random()              # adding noise to the velocity profile
        vel_profile.append(u+noise)

    avg_vel = np.mean(vel_profile)

    x_new = x_car[1] + np.multiply(avg_vel, t) + 0.5*a*np.square(t)

    x_car = x_new
    print(x_car)
    plt.plot(x_car, y_car)
    plt.plot(x_car[1], y_car[1], marker='>', markersize=7)
    plt.pause(0.2)

### for pedestrian
while y_ped[-1] < 0:
    for _ in range(horizon):
        # noise = np.random.random()              # adding noise to the velocity profile
        noise = 0
        vel_profile.append(u_ped+noise)

    avg_vel = np.mean(vel_profile)

    y_new = y_ped[1] + np.multiply(avg_vel, t) + 0.5*a*np.square(t)

    y_ped = y_new

    plt.plot(x_ped, y_ped)
    plt.plot(x_ped[1], y_ped[1], marker='^', markersize=7)
    plt.pause(0.2)

    print(y_ped)
'''