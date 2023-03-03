# ! /usr/bin/env python3

import numpy as np 
import matplotlib.pyplot as plt
from shapely import Point, LineString
import dubins

def get_dubins(start, end, theta0, theta1, step_size = 0.5):
	q0 = (start[0], start[1], theta0)
	q1 = (end[0], end[1], theta1)

	turning_radius = 1.55
	path = dubins.shortest_path(q0, q1, turning_radius)
	
	configurations, _ = path.sample_many(step_size)

	x, y, yaw = np.array([]), np.array([]), np.array([])
	for i in range(len(configurations)):
		x = np.append(x, configurations[i][0])
		y = np.append(y, configurations[i][1])
		if np.pi <= configurations[i][2] <= 2*np.pi:
			yaw = np.append(yaw, 2*np.pi-configurations[i][2])
		else:
			yaw = np.append(yaw, configurations[i][2])
	
	return x, y, yaw

def get_straight_dubins(start, end, theta0, theta1, step_size = 1):
	q0 = (start[0], start[1], theta0)
	q1 = (end[0], end[1], theta1)

	turning_radius = 0.0001

	path = dubins.shortest_path(q0, q1, turning_radius)
	configurations, _ = path.sample_many(step_size)

	x, y, yaw = np.array([]), np.array([]), np.array([])
	for i in range(len(configurations)):
		x = np.append(x, configurations[i][0])
		y = np.append(y, configurations[i][1])
		if np.pi <= configurations[i][2] <= 2*np.pi:
			yaw = np.append(yaw, 2*np.pi-configurations[i][2])
		else:
			yaw = np.append(yaw, configurations[i][2])

	return x, y, yaw


def arr_to_point(x, y):
	point_arr = []
	for i in range(len(x)):
		point_arr.append(Point(x[i], y[i], 0))
	return point_arr

x1, y1, _ = get_straight_dubins([13, -0.9], [2.5, -0.9], np.pi, np.pi)
x9, y9, _ = get_dubins([2.5, -0.9], [0.9, -2.5], np.pi, -np.pi/2)
x6, y6, _ = get_straight_dubins([0.9, -2.5], [0.9, -13], -np.pi/2, -np.pi/2)

x10, y10, _ = get_dubins([2.5, -0.9], [-2.5, -0.9], np.pi, np.pi)				
x3, y3, yaw3 = get_straight_dubins([-2.5, -0.9], [-13, -0.9], np.pi, np.pi)   		
x11, y11, yaw11 = get_dubins([2.5, -0.9], [-0.9, 2.5], np.pi, np.pi/2)				
x7, y7, yaw7 = get_straight_dubins([-0.9, 2.5], [-0.9, 13], np.pi/2, np.pi/2) 		

x5, y5, _ = get_straight_dubins([-0.9, -13], [-0.9, -2.5], np.pi/2, np.pi/2) 	
x14, y14, _ = get_dubins([-0.9, -2.5], [2.5, 0.9], np.pi/2, 0)					
x2, y2, _ = get_straight_dubins([2.5, 0.9], [13, 0.9], 0, 0)             		 

x_1, y_1 = np.hstack((x5, x14, x2)), np.hstack((y5, y14, y2))
x_2_left, y_2_left = np.hstack((x1, x9, x6)), np.hstack((y1, y9, y6))
x_2_straight, y_2_straight = np.hstack((x1, x10, x3)), np.hstack((y1, y10, y3))
x_2_right, y_2_right = np.hstack((x1, x11, x7)), np.hstack((y1, y11, y7))

waypoints_1 = arr_to_point(x_1, y_1)
waypoints_2_left = arr_to_point(x_2_left, y_2_left)
waypoints_2_straight = arr_to_point(x_2_straight, y_2_straight)
waypoints_2_right = arr_to_point(x_2_right, y_2_right)