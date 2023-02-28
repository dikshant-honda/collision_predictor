# ! /usr/bin/env python3

import rospy
import numpy as np 
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
import dubins

def get_dubins(start, end, theta0, theta1, step_size = 0.01):
	q0 = (start[0], start[1], theta0)
	q1 = (end[0], end[1], theta1)

	turning_radius = 1.59
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

def get_straight_dubins(start, end, theta0, theta1, step_size = 0.01):
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

def point_to_arr(lane):
	arr_x, arr_y = [], []
	for i in range(len(lane[0])):
		arr_x.append(lane[0][i].x)
		arr_y.append(lane[0][i].y)
	return arr_x, arr_y

'''
gazebo coordinate system:

	o ----------------> Y
	|
	|
	|
	|
	|
	v
	X
'''

# straight lines information
x1, y1, yaw1 = get_straight_dubins([13, -0.9], [2.5, -0.9], np.pi, np.pi)     		# down
x2, y2, yaw2 = get_straight_dubins([2.5, 0.9], [13, 0.9], 0, 0)             		# down 

x3, y3, yaw3 = get_straight_dubins([-2.5, -0.9], [-13, -0.9], np.pi, np.pi)   		# up
x4, y4, yaw4 = get_straight_dubins([-13, 0.9], [-2.5, 0.9], 0, 0)      	  			# up

x5, y5, yaw5 = get_straight_dubins([-0.9, -13], [-0.9, -2.5], np.pi/2, np.pi/2) 	# left
x6, y6, yaw6 = get_straight_dubins([0.9, -2.5], [0.9, -13], -np.pi/2, -np.pi/2) 	# left

x7, y7, yaw7 = get_straight_dubins([-0.9, 2.5], [-0.9, 13], np.pi/2, np.pi/2) 		# right
x8, y8, yaw8 = get_straight_dubins([0.9, 13], [0.9, 2.5], -np.pi/2, -np.pi/2) 		# right

# intersection information
x9, y9, yaw9 = get_dubins([2.5, -0.9], [0.9, -2.5], np.pi, -np.pi/2)				# down to left
x10, y10, yaw10 = get_dubins([2.5, -0.9], [-2.5, -0.9], np.pi, np.pi)				# down to up
x11, y11, yaw11 = get_dubins([2.5, -0.9], [-0.9, 2.5], np.pi, np.pi/2)				# down to right

x12, y12, yaw12 = get_dubins([-0.9, -2.5], [-2.5, -0.9], np.pi/2, np.pi)			# left to up
x13, y13, yaw13 = get_dubins([-0.9, -2.5], [-0.9, 2.5], np.pi/2, np.pi/2)			# left to right
x14, y14, yaw14 = get_dubins([-0.9, -2.5], [2.5, 0.9], np.pi/2, 0)					# left to down

x15, y15, yaw15 = get_dubins([-2.5, 0.9], [-0.9, 2.5], 0, np.pi/2)					# up to right
x16, y16, yaw16 = get_dubins([-2.5, 0.9], [2.5, 0.9], 0, 0)							# up to down
x17, y17, yaw17 = get_dubins([-2.5, 0.9], [0.9, -2.5], 0, -np.pi/2)					# up to left

x18, y18, yaw18 = get_dubins([0.9, 2.5], [2.5, 0.9], -np.pi/2, 0)					# right to down
x19, y19, yaw19 = get_dubins([0.9, 2.5], [0.9, -2.5], -np.pi/2, -np.pi/2)			# right to left
x20, y20, yaw20 = get_dubins([0.9, 2.5], [-2.5, -0.9], -np.pi/2, np.pi)				# right to up

# possible car routes // not required anymore
# from down
down_to_left = [np.hstack((x1, x9, x6)), np.hstack((y1, y9, y6)), np.hstack((yaw1, yaw9, yaw6))]
down_to_up = [np.hstack((x1, x10, x3)), np.hstack((y1, y10, y3)), np.hstack((yaw1, yaw10, yaw3))]
down_to_right = [np.hstack((x1, x11, x7)), np.hstack((y1, y11, y7)), np.hstack((yaw1, yaw11, yaw7))]

# from left
left_to_up = [np.hstack((x5, x12, x3)), np.hstack((y5, y12, y3)), np.hstack((yaw5, yaw12, yaw3))]
left_to_right = [np.hstack((x5, x13, x7)), np.hstack((y5, y13, y7)), np.hstack((yaw5, yaw13, yaw7))]
left_to_down = [np.hstack((x5, x14, x2)), np.hstack((y5, y14, y2)), np.hstack((yaw5, yaw14, yaw2))]

# from up
up_to_right = [np.hstack((x4, x15, x7)), np.hstack((y4, y15, y7)), np.hstack((yaw4, yaw15, yaw7))]
up_to_down = [np.hstack((x4, x16, x2)), np.hstack((y4, y16, y2)), np.hstack((yaw4, yaw16, yaw2))]
up_to_left = [np.hstack((x4, x17, x6)), np.hstack((y4, y17, y6)), np.hstack((yaw4, yaw17, yaw6))]

# from right
right_to_down = [np.hstack((x8, x18, x2)), np.hstack((y8, y18, y2)), np.hstack((yaw8, yaw18, yaw2))]
right_to_left = [np.hstack((x8, x19, x6)), np.hstack((y8, y19, y6)), np.hstack((yaw8, yaw19, yaw6))]
right_to_up = [np.hstack((x8, x20, x3)), np.hstack((y8, y20, y3)), np.hstack((yaw8, yaw20, yaw3))]

# lane information
lane_1 = [arr_to_point(x1, y1), yaw1]
lane_2 = [arr_to_point(x2, y2), yaw2]
lane_3 = [arr_to_point(x3, y3), yaw3]
lane_4 = [arr_to_point(x4, y4), yaw4]
lane_5 = [arr_to_point(x5, y5), yaw5]
lane_6 = [arr_to_point(x6, y6), yaw6]
lane_7 = [arr_to_point(x7, y7), yaw7]
lane_8 = [arr_to_point(x8, y8), yaw8]
lane_9 = [arr_to_point(x9, y9), yaw9]
lane_10 = [arr_to_point(x10, y10), yaw10]
lane_11 = [arr_to_point(x11, y11), yaw11]
lane_12 = [arr_to_point(x12, y12), yaw12]
lane_13 = [arr_to_point(x13, y13), yaw13]
lane_14 = [arr_to_point(x14, y14), yaw14]
lane_15 = [arr_to_point(x15, y15), yaw15]
lane_16 = [arr_to_point(x16, y16), yaw16]
lane_17 = [arr_to_point(x17, y17), yaw17]
lane_18 = [arr_to_point(x18, y18), yaw18]
lane_19 = [arr_to_point(x19, y19), yaw19]
lane_20 = [arr_to_point(x20, y20), yaw20]

# print(np.hstack((lane_1[0], lane_2[0])))

# plotting
# plt.plot(y1, -x1, 'g')
# plt.plot(y2, -x2, 'g')
# plt.plot(y3, -x3, 'g')
# plt.plot(y4, -x4, 'g')
# plt.plot(y5, -x5, 'g')
# plt.plot(y6, -x6, 'g')
# plt.plot(y7, -x7, 'g')
# plt.plot(y8, -x8, 'g')

# plt.plot(y9, -x9, 'r--')
# plt.plot(y10, -x10, 'r--')
# plt.plot(y11, -x11, 'r--')
# plt.plot(y12, -x12, 'b--')
# plt.plot(y13, -x13, 'b--')
# plt.plot(y14, -x14, 'b--')
# plt.plot(y15, -x15, 'b--')
# plt.plot(y16, -x16, 'b--')
# plt.plot(y17, -x17, 'b--')
# plt.plot(y18, -x18, 'b--')
# plt.plot(y19, -x19, 'b--')
# plt.plot(y20, -x20, 'b--')

# plt.show()
