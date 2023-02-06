# ! /usr/bin/env python3

import numpy as np 
import matplotlib.pyplot as plt
# from geometry_msgs.msg import Point
import dubins

def get_dubins(start, end, theta0, theta1, step_size = 0.02):
	q0 = (start[0], start[1], theta0)
	q1 = (end[0], end[1], theta1)

	turning_radius = 0.4

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

def get_straight_dubins(start, end, theta0, theta1, step_size = 0.02):
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
x1, y1, yaw1 = get_straight_dubins([10, -0.5], [1, -0.5], np.pi, np.pi)     	# down
x2, y2, yaw2 = get_straight_dubins([1, 0.5], [10, 0.5], 0, 0)             		# down 

x3, y3, yaw3 = get_straight_dubins([-1, -0.5], [-10, -0.5], np.pi, np.pi)   	# up
x4, y4, yaw4 = get_straight_dubins([-10, 0.5], [-1, 0.5], 0, 0)      	  		# up

x5, y5, yaw5 = get_straight_dubins([-0.5, -10], [-0.5, -1], np.pi/2, np.pi/2) 	# left
x6, y6, yaw6 = get_straight_dubins([0.5, -1], [0.5, -10], -np.pi/2, -np.pi/2) 	# left

x7, y7, yaw7 = get_straight_dubins([-0.5, 1], [-0.5, 10], np.pi/2, np.pi/2) 	# right
x8, y8, yaw8 = get_straight_dubins([0.5, 10], [0.5, 1], -np.pi/2, -np.pi/2) 	# right

# intersection information
x9, y9, yaw9 = get_dubins([1, -0.5], [0.5, -1], np.pi, -np.pi/2)				# down to left
x10, y10, yaw10 = get_dubins([1, -0.5], [-1, -0.5], np.pi, np.pi)				# down to up
x11, y11, yaw11 = get_dubins([1, -0.5], [-0.5, 1], np.pi, np.pi/2)				# down to right

x12, y12, yaw12 = get_dubins([-0.5, -1], [-1, -0.5], np.pi/2, np.pi)			# left to up
x13, y13, yaw13 = get_dubins([-0.5, -1], [-0.5, 1], np.pi/2, np.pi/2)			# left to right
x14, y14, yaw14 = get_dubins([-0.5, -1], [1, 0.5], np.pi/2, 0)					# left to down

x15, y15, yaw15 = get_dubins([-1, 0.5], [-0.5, 1], 0, np.pi/2)					# up to right
x16, y16, yaw16 = get_dubins([-1, 0.5], [1, 0.5], 0, 0)							# up to down
x17, y17, yaw17 = get_dubins([-1, 0.5], [0.5, -1], 0, -np.pi/2)					# up to left

x18, y18, yaw18 = get_dubins([0.5, 1], [1, 0.5], -np.pi/2, 0)					# right to down
x19, y19, yaw19 = get_dubins([0.5, 1], [0.5, -1], -np.pi/2, -np.pi/2)			# right to left
x20, y20, yaw20 = get_dubins([0.5, 1], [-1, -0.5], -np.pi/2, np.pi)				# right to up

# vehicle trajectories
# car 1 path
# x_car_1 = np.hstack((x3, x14, x6, x18, x8))
# y_car_1 = np.hstack((y3, y14, y6, y18, y8))  
# car_yaw_1 =  np.hstack((yaw3, yaw14, yaw6, yaw18, yaw8))

# # car 2 path 
# x_car_2 = np.hstack((x7[::-1], x19[::-1], x6[::-1], x14[::-1], x3[::-1]))
# y_car_2 = np.hstack((y7[::-1], y19[::-1], y6[::-1], y14[::-1], y3[::-1]))
# car_yaw_2 = np.hstack((np.add(yaw7, np.pi/2), np.add(yaw19, np.pi/4+np.pi/2), np.add(yaw6, np.pi), np.add(yaw14, np.pi), np.add(yaw3, np.pi)))

# # car 3 path
# x_car_3 = np.hstack((x4, x17, x6, x18, x8))
# y_car_3 = np.hstack((y4, y17, y6, y18, y8))
# car_yaw_3 = np.hstack((np.add(yaw4, np.pi), np.add(yaw17[::-1], 3*np.pi/2), yaw6, yaw18, yaw8))

# # car 4 path
# x_car_4 = np.hstack((x5[::-1], x13[::-1], x3[::-1], x11[::-1], x2[::-1]))
# y_car_4 = np.hstack((y5[::-1], y13[::-1], y3[::-1], y11[::-1], y2[::-1]))
# car_yaw_4 = np.hstack((yaw5, np.add(yaw13, np.pi/2), np.add(yaw3, np.pi), np.add(yaw11, np.pi/2), yaw2))

# # car 5 path
# x_car_5 = np.hstack((x1, x9, x3, x12, x4[::-1]))
# y_car_5 = np.hstack((y1, y9, y3, y12, y4[::-1]))
# car_yaw_5 = np.hstack((yaw1, yaw9, yaw3, yaw12, yaw4[::-1]))

# converting car path into geometry_msgs/Point
# car_1_route = []
# for i in range(len(x_car_1)):
#     car_1_route.append(Point(x_car_1[i], y_car_1[i], 0))

# car_2_route = []
# for i in range(len(x_car_2)):
#     car_2_route.append(Point(x_car_2[i], y_car_2[i], 0))

# car_3_route = []
# for i in range(len(x_car_3)):
#     car_3_route.append(Point(x_car_3[i], y_car_3[i], 0))

# car_4_route = []
# for i in range(len(x_car_4)):
#     car_4_route.append(Point(x_car_4[i], y_car_4[i], 0))

# car_5_route = []
# for i in range(len(x_car_5)):
#     car_5_route.append(Point(x_car_5[i], y_car_5[i], 0))

# plotting
# plt.plot(y1, -x1, 'g')
# plt.plot(y2, -x2, 'g')
# plt.plot(y3, -x3, 'g')
# plt.plot(y4, -x4, 'g')
# plt.plot(y5, -x5, 'g')
# plt.plot(y6, -x6, 'g')
# plt.plot(y7, -x7, 'g')
# plt.plot(y8, -x8, 'g')

# plt.plot(y9, -x9, 'b--')
# plt.plot(y10, -x10, 'b--')
# plt.plot(y11, -x11, 'b--')
# plt.plot(y12, -x12, 'b--')
# plt.plot(y13, -x13, 'b--')
# plt.plot(y14, -x14, 'b--')
# plt.plot(y15, -x15, 'b--')
# plt.plot(y16, -x16, 'b--')
# plt.plot(y17, -x17, 'b--')
# plt.plot(y18, -x18, 'b--')
# plt.plot(y19, -x19, 'b--')
# plt.plot(y20, -x20, 'b--')

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

plt.show()
