import numpy as np
import matplotlib.pyplot as plt
import math
import warnings

'''
TO DO: 
-> modify the stopping condition
1. add the stopping condition of the collision ball instead of only one direction   # DONE
2. what if the vehicle have actually passed the collision scenario                  # DONE
-> dynamics update
1. include acceleration in the dynamics                                             # DONE
2. check the velocity profile update system                                         # DONE
3. angular velocity and steering angle dynamics                                     # DONE
4. inclusion of curvy roads and intersection scenario                               # DONE
5. different time horizons for ego vehicle and traffic participants 
6. head-to-head and lane merge scenario                 
-> scenario change
1. think about the motion of the traffic participants in both direction             # DONE
2. adding vehicles with angular velocities                                          
3. adding bounds of the vehicle motion
4. changing the vehicle from a point object to a box
5. adding uncertainities into the velocity profile of the traffic participants      # DONE
6. add more traffic participants and check their collision probability              # DONE
7. simulate everything on the gazebo                
8. fusion from camera and radar data            
-> plotting
1. plotting the velocity profiles                                                   # DONE
2. add road boundaries 
-> coding practices
1. add agent type to the step function
2. improve the number of check in close function
3. improve the collision function
4. add noise function
5. change position to list instead of individual int
6. improve the way of adding the vehicle
7. add a vehicle registry class or function
'''

horizon = 10                                        # length of velocity profile 
future_horizon = 10                                 # number of time steps
max_time = 5                                        # future time horizon
t = 0.5                                             # time step
tol = 0.5                                           # tolerance value for proximity check
order = 4                                           # degree of the polynomial fit curve
max_dec = 0.3                                       # max deceleration value
agent = ["ego", "pedestrian", "cyclist", "traffic"] # type of agents

# for ego
agent_type = agent[0]
start_pos_car = -5
x_ego = np.linspace(start_pos_car, -1, horizon)
y_ego = np.linspace(3, 3, horizon)
ego_pos = [x_ego, y_ego]
u_car = 5                                           # initial velocity of the ego car    
a_car = 0.5                                         # acceleration of the ego car

# for pedestrian
agent_type = agent[1]
start_pos_ped = -2
x_ped = np.linspace(7,7,horizon)
y_ped = np.linspace(start_pos_ped, -1, horizon)
ped_pos = [x_ped, y_ped]
u_ped = 0.3                                         # initial velocity of the pedestrian
a_ped = 0.05                                        # acceleration of the pedestrian

# for car moving in diagonal direction
agent_type = agent[3]
start_pos_car_1_x = -5
x_pt = np.linspace(start_pos_car_1_x, 0, horizon)
theta = np.pi/12
x_car_1 = x_pt*np.cos(theta)
y_car_1 = x_pt*np.sin(theta)
car1_pos = [x_car_1, y_car_1]
u_car_1 = 1.5                                       # initial velocity of the traffic car 1
a_car_1 = 0.2                                       # acceleration of the traffic car 1

# for car moving on curves
agent_type = agent[3]
start_pos_car_2_x = -5
x_car_2 = np.linspace(start_pos_car_2_x,1,horizon)     
a = 0.05
b = -0.03
c = 0.5
d = -7
# y_car_2 = np.multiply(a, np.power(x_car_2,3))+np.multiply(b, np.power(x_car_2,2))+np.multiply(c,x_car_2)+d
y_car_2 = np.multiply(b, np.power(x_car_2,2)) + np.multiply(c,x_car_2) 
# y_car_2 = np.sin(x_car_2)
car2_pos = [x_car_2, y_car_2]
u_car_2 = 0.5                                       # initial velocity of the traffic car 2
a_car_2 = 0.05                                      # acceleration of the traffic car 2

# plotting 
plt.title("Collision Predictor")
plt.xlabel("x")
plt.ylabel("y")
plt.plot(x_ego[-1], y_ego[-1], marker='s', markersize=5, label='ego')
plt.plot(x_ped[-1], y_ped[-1], marker='s', markersize=5, label='pedestrian')
plt.plot(x_car_1[-1], y_car_1[-1], marker = 's', markersize = 5, label='veh 1')
plt.plot(x_car_2[-1], y_car_2[-1], marker = 's', markersize = 5, label='veh 2')
plt.legend()
plt.xlim(-5,15)
plt.ylim(-5,5) 

# proximity check function
def close(pos1, pos2):
    for i in range(len(pos1[0])):
        for j in range(len(pos2[0])):
            if np.sqrt((pos1[0][i]-pos2[0][j])**2 + (pos1[1][i]-pos2[1][j])**2) < tol:
                return True
    return False

# curve fitting to approximate future trajectory
def fit(x, y, order):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', np.RankWarning)
        z = np.polyfit(x, y, order)
        f = np.poly1d(z)
    return f

# stopping function
def collision(ego_future, ped_future, car1_future, car2_future):    
    if close(ego_future, ped_future):
        print("collision with pedestrian")
        print("!!STOP!!")
        return False   
    if close(ego_future, car1_future):
        print("collision with diagonally moving car")
        print("!!STOP!!")
        return False
    if close(ego_future, car2_future):
        print("collision with car moving on curvy path")
        print("!!STOP!!")
        return False
    return True

def step(pos):
    # average velocity estimation
    vel_profile = []
    acc_profile = []
    for _ in range(horizon):
        vel_profile.append(u_car_2+np.random.random())            # adding noise
        acc_profile.append(a_car_2+np.random.choice([-1,1])*np.random.random()/2)
    # pedestrian case
    vel_profile_ped = []
    acc_profile_ped = []
    for _ in range(horizon):
        vel_profile_ped.append(u_car_2+np.random.random()/10)            # adding noise
        acc_profile_ped.append(a_car_2+np.random.choice([-1,1])*np.random.random()/5)
    
    avg_vel = np.mean(vel_profile)
    avg_acc = np.mean(acc_profile)
    avg_vel_ped = np.mean(vel_profile_ped)
    avg_acc_ped = np.mean(acc_profile_ped)

    # future trajectory estimation
    x, y = pos[0], pos[1]
    f = fit(x, y, order)
    if x[-1]-x[0] == 0 and y[-1]-y[0] != 0:
        theta = np.pi/2
    elif x[-1]-x[0] != 0 and y[-1]-y[0] == 0:
        theta = 0
    else:
        theta = np.arctan((y[-1]-y[-5])/(x[-1]-x[-5]))

    s = avg_vel**2 / (2*max_dec)
    # final_x = x[-1] + avg_vel*t + 0.5*avg_acc*(future_time_step**2)
    if math.isclose(theta, np.pi/2):
        future_x = np.linspace(x[-1], x[-1], future_horizon)
        future_y = y[-1] + np.linspace(y[-1],y[-1]+s, future_horizon)
        x_new = x + avg_vel_ped*np.cos(theta)*t + 0.5*avg_acc_ped*np.cos(theta)*(t**2)
        y_new = y + avg_vel_ped*np.sin(theta)*t + 0.5*avg_acc_ped*np.sin(theta)*(t**2)
        updated_pos = [x_new, y_new]
        updated_future_pos = [future_x, future_y]
        return updated_pos, updated_future_pos
    else:
        future_x = np.linspace(x[-1], x[-1]+s*np.cos(theta), future_horizon)
        future_y = f(future_x)

    # update 
    x_new = x + avg_vel*np.cos(theta)*t + 0.5*avg_acc*np.cos(theta)*(t**2)
    y_new = y + avg_vel*np.sin(theta)*t + 0.5*avg_acc*np.sin(theta)*(t**2)

    updated_pos = [x_new, y_new]
    updated_future_pos = [future_x, future_y]
    return updated_pos, updated_future_pos

# initialization for the future trajectories
ego_future = ego_pos
ped_future = ped_pos
car1_future = car1_pos
car2_future = car2_pos

# main function
if __name__ == '__main__':
    while collision(ego_future, ped_future, car1_future, car2_future):
        # straight moving car
        ego_pos, ego_future = step(ego_pos)

        # pedestrian dynamics
        ped_pos, ped_future = step(ped_pos)
        
        # diagonally moving car dynamics
        car1_pos, car1_future = step(car1_pos)

        # curvy car dynamics
        car2_pos, car2_future = step(car2_pos)

        # plotting
        # past trajectories
        # plt.plot(ego_pos[0], ego_pos[1])
        plt.plot(ego_pos[0][-1], ego_pos[1][-1], marker='s', markersize=5, label='ego')
        # plt.plot(ped_pos[0], ped_pos[1])
        plt.plot(ped_pos[0][-1], ped_pos[1][-1], marker='s', markersize=5, label='pedestrian')
        # plt.plot(car1_pos[0], car1_pos[1])
        plt.plot(car1_pos[0][-1], car1_pos[1][-1], marker = 's', markersize = 5, label='veh 1')
        # plt.plot(car2_pos[0], car2_pos[1], marker = ',', markersize = 0.5)
        plt.plot(car2_pos[0][-1], car2_pos[1][-1], marker = 's', markersize = 5, label='veh 2')

        #future trajectories
        plt.plot(ego_future[0], ego_future[1], marker='>', markersize=2)
        plt.plot(ped_future[0], ped_future[1], marker='^', markersize=3)
        plt.plot(car1_future[0], car1_future[1], marker = '*', markersize = 2)
        plt.plot(car2_future[0], car2_future[1], marker = 'o', markersize = 2)
        
        plt.pause(0.5)

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