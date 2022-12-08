import numpy as np
import matplotlib.pyplot as plt

'''
TO DO: 
-> modify the stopping condition
1. add the stopping condition of the collision ball instead of only one direction   # DONE
2. what if the vehicle have actually passed the collision scenario                  # DONE
-> dynamics update
1. include acceleration in the dynamics                                             # DONE
2. check the velocity profile update system                                         # DONE
3. angular velocity and steering angle dynamics
4. inclusion of curvy roads and intersection scenario
5. different time horizons for ego vehicle and traffic participants 
6. head-to-head and lane merge scenario 
-> scenario change
1. think about the motion of the traffic participants in both direction
2. adding vehicles with angular velocities 
3. adding bounds of the vehicle motion
4. changing the vehicle from a point object to a box
5. adding uncertainities into the velocity profile of the traffic participants
6. add more traffic participants and check their collision probability
7. simulate everything on the gazebo
8. fusion from camera and radar data
-> plotting
1. plotting the velocity profiles                                                   # DONE
2. add road boundaries 
'''

horizon = 10                                    # length of velocity profile 
future_horizon = 10                             # number of time steps
max_time = 5                                    # future time horizon
t = np.linspace(0, max_time,future_horizon)     # time steps
tol = 0.3                                       # tolerance value for proximity check

# for car
start_pos_car = 0
x_car = [start_pos_car]*future_horizon
u_car = 2.5                                     # initial velocity of the car    
a_car = 0.5                                     # acceleration of the car

# for car moving in diagonal direction
start_pos_car_1_x = 5
start_pos_car_1_y = -10
future_horizon_car_1 = 10
x_car_1 = [start_pos_car_1_x]*future_horizon_car_1
y_car_1 = [start_pos_car_1_y]*future_horizon_car_1
theta = np.pi/5
u_car_1 = 1.5
a_car_1 = 0.2

# for pedestrian
start_pos_ped = -5
future_horizon_ped = 10
y_ped = [start_pos_ped]*future_horizon_ped
u_ped = 0.4                                     # initial velocity of the pedestrian
a_ped = 0.1                                     # acceleration of the pedestrian

# plotting 
y_car = [0]*future_horizon 
x_ped = [20]*future_horizon
plt.title("Collision Predictor")
plt.xlabel("x")
plt.ylabel("y")
# plt.xlim(-5,35)
# plt.ylim(-10,15)

# proximity check function
def close(a, b):
    if np.isclose(a, b, atol = tol).any():
        return True
    return False

# stopping function
def stop(x_car, y_car, x_ped, y_ped, x_car_1, y_car_1):
    if (close(x_car, x_ped) and close(y_car, y_ped)) or (close(x_car, x_car_1) and close(y_car, y_car_1)) :
        print("!!STOP!!")
        return False
    return True

### MERGE
while stop(x_car,y_car,x_ped,y_ped,x_car_1,y_car_1):
    vel_profile_car = []
    acc_profile_car = []
    vel_profile_ped = []
    acc_profile_ped = []
    vel_profile_car_1 = []
    acc_profile_car_1 = []
    for _ in range(horizon):
        noise_car = np.random.random()          # noise in the velocity profile of car
        vel_profile_car.append(u_car+noise_car)
        acc_profile_car.append(a_car+np.random.choice([-1,1])*np.random.random()/2)
        noise_ped = np.random.random()          # noise in the velocity profile of pedestrian
        vel_profile_ped.append(u_ped+noise_ped/10)
        acc_profile_ped.append(a_ped+np.random.choice([-1,1])*np.random.random()/10)
        noise_ego = np.random.random()
        vel_profile_car_1.append(u_car_1+noise_ego)
        acc_profile_car_1.append(a_car_1+np.random.choice([-1,1])*np.random.random()/2)
    
    # car dynamics
    avg_vel_car = np.mean(vel_profile_car)
    avg_acc_car = np.mean(acc_profile_car)
    # update the x_car[1] by the actual position of the car
    x_new = x_car[1] + np.multiply(avg_vel_car, t) + np.multiply(0.5*avg_acc_car,np.square(t))
    x_car = x_new

    # pedestrian dynamics
    avg_vel_ped = np.mean(vel_profile_ped)
    avg_acc_ped = np.mean(acc_profile_ped)
    # update the y_ped[1] by the actual position of the pedestrian
    y_new = y_ped[1] + np.multiply(avg_vel_ped, t) + np.multiply(0.5*avg_acc_ped, np.square(t))
    y_ped = y_new
    
    # ego car dynamics
    avg_vel_ego = np.mean(vel_profile_car_1)
    avg_acc_ego = np.mean(acc_profile_car_1)
    # update according to actual position
    x_new = x_car_1[1] + np.multiply(avg_vel_ego*np.cos(theta), t) + np.multiply(0.5*avg_acc_ego*np.cos(theta), np.square(t))
    y_new = y_car_1[1] + np.multiply(avg_vel_ego*np.sin(theta), t) + np.multiply(0.5*avg_acc_ego*np.sin(theta), np.square(t))
    x_car_1 = x_new
    y_car_1 = y_new

    # plotting
    plt.plot(x_car, y_car)
    plt.plot(x_car[1], y_car[1], marker='>', markersize=5)
    plt.plot(x_ped, y_ped)
    plt.plot(x_ped[1], y_ped[1], marker='^', markersize=5)
    plt.plot(x_car_1, y_car_1)
    plt.plot(x_car_1[1], y_car_1[1], marker = 'o', markersize = 5)
    # plotting the velocity profile
    # time = [i for i in range(horizon)]
    # plt.plot(time, vel_profile_car)
    # plt.plot(time, vel_profile_ped)

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