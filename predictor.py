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
t = 0.5                                         # time step
tol = 0.5                                       # tolerance value for proximity check
order = 4                                       # degree of the polynomial fit curve

# for car
start_pos_car = -10
x_car = np.linspace(start_pos_car, -5, horizon)
y_car = np.linspace(0, 0, horizon)
u_car = 2.5                                     # initial velocity of the car    
a_car = 0.5                                     # acceleration of the car

# for pedestrian
start_pos_ped = -5
x_ped = np.linspace(5,5,horizon)
y_ped = np.linspace(start_pos_ped, -3, horizon)
u_ped = 0.3                                     # initial velocity of the pedestrian
a_ped = 0.05                                    # acceleration of the pedestrian

# for car moving in diagonal direction
start_pos_car_1_x = -5
start_pos_car_1_y = 10
x_pt = np.linspace(start_pos_car_1_x, 10, horizon)
theta = -np.pi/5
x_car_1 = x_pt*np.cos(theta)
y_car_1 = x_pt*np.sin(theta)
u_car_1 = 1.5
a_car_1 = 0.2

# for car moving on curves
start_pos_car_2_x = -5
start_pos_car_2_y = -10
x_car_2 = np.linspace(start_pos_car_2_x,1,horizon)    ## update this 
a = 0.05
b = 0.02
c = 0.5
d = 0.005
y_car_2 = np.multiply(a, np.power(x_car_2,3))+np.multiply(b, np.power(x_car_2,2))+np.multiply(c,x_car_2)+d
# y_car_2 = np.multiply(b, np.power(x_car_2,2)) + np.multiply(c,x_car_2)
u_car_2 = 1
a_car_2 = 0.3

# plotting 
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

# curve fitting to approximate future trajectory
def fit(x, y, order):
    z = np.polyfit(x, y, order)
    f = np.poly1d(z)
    return f

# stopping function
def stop(x_car, y_car, x_ped, y_ped, x_car_1, y_car_1):
    if close(x_car, x_ped) and close(y_car, y_ped):
        print("collision with pedestrian")
        print("!!STOP!!")
        return False
    if close(x_car, x_car_1) and close(y_car, y_car_1) :
        print("collision with other car")
        print("!!STOP!!")
        return False
    return True

def dynamics(x, y):
    # future trajectory estimation
    f = fit(x, y, order)
    future_x = np.linspace(x[0], x[-1]+horizon/2)
    future_y = f(future_x)

    # average velocity estimation
    vel_profile = []
    acc_profile = []
    for _ in range(horizon):
        vel_profile.append(u_car_2+np.random.random())            # adding noise
        acc_profile.append(a_car_2+np.random.choice([-1,1])*np.random.random()/2)
    avg_vel = np.mean(vel_profile)
    avg_acc= np.mean(acc_profile)

    # update 
    if x[-1]-x[0] == 0 and y[-1]-y[0] != 0:
        print("is it ped")
        theta = np.pi/2
    elif x[-1]-x[0] != 0 and y[-1]-y[0] == 0:
        theta = 0
    else:
        theta = np.arctan((y[-1]-y[0])/(x[-1]-x[0]))
    x_new = x + avg_vel*np.cos(theta)*t + 0.5*avg_acc*np.cos(theta)*(t**2)
    y_new = y + avg_vel*np.sin(theta)*t + 0.5*avg_acc*np.sin(theta)*(t**2)
    return x_new, y_new, future_x, future_y

### MERGE
while True:# stop(x_car,y_car,x_ped,y_ped,x_car_1,y_car_1):
    # straight moving car
    x_car, y_car, x_car_future, y_car_future = dynamics(x_car, y_car)

    # pedestrian dynamics
    x_ped, y_ped, x_ped_future, y_ped_future = dynamics(x_ped, y_ped)
    
    # diagonally moving car dynamics
    x_car_1, y_car_1, x_car_future_1, y_car_future_1 = dynamics(x_car_1, y_car_1)

    # curvy car dynamics
    x_car_2, y_car_2, x_car_future_2, y_car_future_2 = dynamics(x_car_2, y_car_2)

    # plotting
    plt.plot(x_car, y_car)
    plt.plot(x_car[0], y_car[0], marker='>', markersize=5)
    plt.plot(x_ped, y_ped)
    plt.plot(x_ped[0], y_ped[0], marker='^', markersize=5)
    plt.plot(x_car_1, y_car_1)
    plt.plot(x_car_1[0], y_car_1[0], marker = 'o', markersize = 5)
    plt.plot(x_car_2, y_car_2)
    plt.plot(x_car_2[0], y_car_2[0], marker = '*', markersize = 5)
    plt.plot(x_car_future[-1], y_car_future[-1], marker='s', markersize=5)
    plt.plot(x_ped_future[-1], y_ped_future[-1], marker='s', markersize=5)
    plt.plot(x_car_future_1[-1], y_car_future_1[-1], marker = 's', markersize = 5)
    plt.plot(x_car_future_2[-1], y_car_future_2[-1], marker = 's', markersize = 5)
    # plotting the velocity profile
    # time = [i for i in range(horizon)]
    # plt.plot(time, vel_profile_acr)
    # plt.plot(time, vel_profile_ped)

    plt.pause(1)

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