# ! /usr/bin/env python3

import numpy as np 
import matplotlib.pyplot as plt
# from Bezier import Bezier
# from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Point
import dubins

def get_lanes(start, end, steps=100):
    x = np.linspace(start[0], end[0], steps)
    y = np.linspace(start[1], end[1], steps)
    return x, y

def get_spline(start, end, theta0, theta1, steps=100): 
    # change the format of input to the function to point
	t = np.linspace(0, 1, steps) 

	dx0 = np.cos(theta0) 
	dy0 = np.sin(theta0)
	dx1 = np.cos(theta1) 
	dy1 = np.sin(theta1)

	t0 = 0
	t1 = 1

	Ax = np.asarray([[1, t0,   t0**2,   t0**3],  # x  @ 0
					[0, 1,  2*t0,    3*t0**2],  # x' @ 0
					[1, t1,   t1**2,   t1**3],  # x  @ 1
					[0, 1,  2*t1,    3*t1**2]]) # x' @ 1

	X = np.asarray([start[0], dx0, end[0], dx1]).transpose()
	bx = np.linalg.solve(Ax, X)

	Ay = np.asarray([[1, t0,   t0**2,   t0**3],  # x  @ 0
					[0, 1,  2*t0,    3*t0**2],  # x' @ 0
					[1, t1,   t1**2,   t1**3],  # x  @ 1
					[0, 1,  2*t1,    3*t1**2]]) # x' @ 1
	Y = np.asarray([start[1], dy0, end[1], dy1]).transpose()
	by = np.linalg.solve(Ay, Y)

	x = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(),bx)
	y = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(),by)

	return x, y

def get_dubins(start, end, theta0, theta1, step_size = 0.02):
	q0 = (start[0], start[1], theta0)
	q1 = (end[0], end[1], theta1)

	turning_radius = 0.7

	path = dubins.shortest_path(q0, q1, turning_radius)
	configurations, _ = path.sample_many(step_size)

	x, y, yaw = [], [], []
	for i in range(len(configurations)):
		x.append(configurations[i][0])
		y.append(configurations[i][1])
		if np.pi <= configurations[i][2] <= 2*np.pi:
			yaw.append(2*np.pi-configurations[i][2])
		else:
			yaw.append(configurations[i][2])

	return x, y, yaw

def get_straight_dubins(start, end, theta0, theta1, step_size = 0.02):
	q0 = (start[0], start[1], theta0)
	q1 = (end[0], end[1], theta1)

	turning_radius = 0.0001

	path = dubins.shortest_path(q0, q1, turning_radius)
	configurations, _ = path.sample_many(step_size)

	x, y, yaw = [], [], []
	for i in range(len(configurations)):
		x.append(configurations[i][0])
		y.append(configurations[i][1])
		if np.pi <= configurations[i][2] <= 2*np.pi:
			yaw.append(2*np.pi-configurations[i][2])
		else:
			yaw.append(configurations[i][2])

	return x, y, yaw

#  straight lanes
# x1, y1 = get_lanes([-6.3,-1.0],[-6.3,-8])     # down right
# x2, y2 = get_lanes([-6.3,8], [-6.3,1])      # down left
# x3, y3 = get_lanes([-5,0], [-1,0])      # down center
# x4, y4 = get_lanes([0,5], [0,1])        # center left
# x5, y5 = get_lanes([0,-1], [0,-5])       # center right
# x6, y6 = get_lanes([1,0], [5.3,0])      # up center
# x7, y7 = get_lanes([6.5,-0.5], [10,-4]) # up right
# x8, y8 = get_lanes([6.5,0.5], [10,4])   # up left

x1, y1, yaw1 = get_straight_dubins([-6.3,-8], [-6.3,-1], np.pi/2, np.pi/2)     # down right
x2, y2, yaw2 = get_straight_dubins([-6.3,8], [-6.3,1], -np.pi/2, -np.pi/2)      # down left
x3, y3, yaw3 = get_straight_dubins([-5,0], [-1,0], 0, 0)      # down center
x4, y4, yaw4 = get_straight_dubins([0,5], [0,1], -np.pi/2, -np.pi/2)        # center left
x5, y5, yaw5 = get_straight_dubins([0,-1], [0,-5], -np.pi/2, -np.pi/2)       # center right
x6, y6, yaw6 = get_straight_dubins([1,0], [5.3,0], 0, 0)      # up center
x7, y7, yaw7 = get_straight_dubins([6.5,-0.5], [10,-4], np.pi/4, np.pi/4) # up right
x8, y8, yaw8 = get_straight_dubins([6.5,0.5], [10,4], -np.pi/4, -np.pi/4)   # up left

# splines
# # T intersection
# x9, y9 = get_spline([-6.3,-1.0], [-5,0], np.pi/2,np.pi/6)
# x10, y10 = get_spline([-6.3,-1], [-6.3,1], np.pi/2, -np.pi/2)
# x11, y11 = get_spline([-6.3,1], [-5,0], -np.pi/2, np.pi/6)

# # X intersection
# x12, y12 = get_spline([-1,0], [0,1], 0, np.pi/2)
# x13, y13 = get_spline([-1,0], [0,-1], 0, -np.pi/2)
# x14, y14 = get_spline([-1,0], [1,0], 0, 0)
# x15, y15 = get_spline([0,-1], [1,0], np.pi/2, 0)
# x16, y16 = get_spline([0,1], [0,-1], -np.pi/2, -np.pi/2)
# x17, y17 = get_spline([0,1], [1,0], -np.pi/2, 0)

# # Y intersection
# x18, y18 = get_spline([5.3,0], [6.5, 0.5], 0, np.pi/4)
# x19, y19 = get_spline([6.5,-0.5], [5.3,0], 0, np.pi/4+np.pi/2)
# x20, y20 = get_spline([6.5,0.5], [6.5,-0.5], -(np.pi/4+np.pi/2), -np.pi/4)

# dubins
x9, y9, yaw9 = get_dubins([-6.3,-1.0], [-5,0], np.pi/2, 0)
x10, y10, yaw10 = get_dubins([-6.3,-1], [-6.3,1], np.pi/2, np.pi/2)
x11, y11, yaw11 = get_dubins([-6.3,1], [-5,0], -np.pi/2, 0)

# X intersection
x12, y12, yaw12 = get_dubins([-1,0], [0,1], 0, np.pi/2)
x13, y13, yaw13 = get_dubins([-1,0], [0,-1], 0, -np.pi/2)
x14, y14, yaw14 = get_dubins([-1,0], [1,0], 0, 0)
x15, y15, yaw15 = get_dubins([0,-1], [1,0], np.pi/2, 0)
x16, y16, yaw16 = get_dubins([0,1], [0,-1], -np.pi/2, -np.pi/2)
x17, y17, yaw17 = get_dubins([0,1], [1,0], -np.pi/2, 0)

# Y intersection
x18, y18, yaw18 = get_dubins([5.3,0], [6.5, 0.5], 0, np.pi/4)
x19, y19, yaw19 = get_dubins([5.3,0], [6.5,-0.5], 0, -np.pi/4)
x20, y20, yaw20 = get_dubins([6.5,0.5], [6.5,-0.5], -(np.pi/4+np.pi/2), -np.pi/4)

# vehicle trajectories
# car 1 path
x_car_1 = np.hstack((x3, x14, x6, x18, x8))
y_car_1 = np.hstack((y3, y14, y6, y18, y8))  
car_yaw_1 =  np.hstack((yaw3, yaw14, yaw6, yaw18, yaw8))

# car 2 path 
x_car_2 = np.hstack((x7[::-1], x19[::-1], x6[::-1], x14[::-1], x3[::-1]))
y_car_2 = np.hstack((y7[::-1], y19[::-1], y6[::-1], y14[::-1], y3[::-1]))
car_yaw_2 = np.hstack((np.add(yaw7, np.pi/2), np.add(yaw19, np.pi/4+np.pi/2), np.add(yaw6, np.pi), np.add(yaw14, np.pi), np.add(yaw3, np.pi)))

# car 3 path
x_car_3 = np.hstack((x4, x17, x6, x18, x8))
y_car_3 = np.hstack((y4, y17, y6, y18, y8))
car_yaw_3 = np.hstack((np.add(yaw4, np.pi), np.add(yaw17[::-1], 3*np.pi/2), yaw6, yaw18, yaw8))

# car 4 path
x_car_4 = np.hstack((x5[::-1], x13[::-1], x3[::-1], x11[::-1], x2[::-1]))
y_car_4 = np.hstack((y5[::-1], y13[::-1], y3[::-1], y11[::-1], y2[::-1]))
car_yaw_4 = np.hstack((yaw5, np.add(yaw13, np.pi/2), np.add(yaw3, np.pi), np.add(yaw11, np.pi/2), yaw2))

# car 5 path
x_car_5 = np.hstack((x1, x9, x3, x12, x4[::-1]))
y_car_5 = np.hstack((y1, y9, y3, y12, y4[::-1]))
car_yaw_5 = np.hstack((yaw1, yaw9, yaw3, yaw12, yaw4[::-1]))

# converting car path into geometry_msgs/Point
car_1_route = []
for i in range(len(x_car_1)):
    car_1_route.append(Point(x_car_1[i], y_car_1[i], 0))

car_2_route = []
for i in range(len(x_car_2)):
    car_2_route.append(Point(x_car_2[i], y_car_2[i], 0))

car_3_route = []
for i in range(len(x_car_3)):
    car_3_route.append(Point(x_car_3[i], y_car_3[i], 0))

car_4_route = []
for i in range(len(x_car_4)):
    car_4_route.append(Point(x_car_4[i], y_car_4[i], 0))

car_5_route = []
for i in range(len(x_car_5)):
    car_5_route.append(Point(x_car_5[i], y_car_5[i], 0))

# plotting
# plt.plot(x1, y1, 'k')
# plt.plot(x2, y2, 'k')  
# plt.plot(x3, y3, 'k')  
# plt.plot(x4, y4, 'k')  
# plt.plot(x5, y5, 'k')  
# plt.plot(x6, y6, 'k')  
# plt.plot(x7, y7, 'k')  
# plt.plot(x8, y8, 'k')

# plt.plot(x9, y9, 'b--')  
# plt.plot(x10, y10, 'b--')  
# plt.plot(x11, y11, 'b--')
# plt.plot(x12, y12, 'b--')
# plt.plot(x13, y13, 'b--')
# plt.plot(x14, y14, 'b--')
# plt.plot(x15, y15, 'b--')
# plt.plot(x16, y16, 'b--')
# plt.plot(x17, y17, 'b--')
# plt.plot(x18, y18, 'b--')
# plt.plot(x19, y19, 'b--')
# plt.plot(x20, y20, 'b--')

# plt.plot(x_car_1, y_car_1)
# plt.plot(x_car_2, y_car_2)
# plt.plot(x_car_3, y_car_3)
# plt.plot(x_car_4, y_car_4)
# plt.plot(x_car_5, y_car_5)

# print(x_car_1, y_car_1, yaw_car_1)
# print(x_car_2, y_car_2, yaw_car_2)
# print(x_car_3, y_car_3, yaw_car_3)
# print(x_car_4, y_car_4, car_yaw_4)
# print(x_car_5, y_car_5, yaw_car_5)

# plt.show()
