import numpy as np 
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point

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

# if __name__ == '__main__':
#  straight lanes
x1, y1 = get_lanes([-6,-1],[-6,-8])     # down right
x2, y2 = get_lanes([-6,8], [-6,1])      # down left
x3, y3 = get_lanes([-5,0], [-1,0])      # down center
x4, y4 = get_lanes([0,5], [0,1])        # center left
x5, y5 = get_lanes([0,-1], [0,-5])       # center right
x6, y6 = get_lanes([1,0], [5.5,0])      # up center
x7, y7 = get_lanes([6.5,-0.5], [10,-4]) # up right
x8, y8 = get_lanes([6.5,0.5], [10,4])   # up left

# splines
# T intersection
x9, y9 = get_spline([-6,-1], [-5,0], np.pi/2,0)
x10, y10 = get_spline([-6,-1], [-6,1], np.pi/2, -np.pi/2)
x11, y11 = get_spline([-6,1], [-5,0], -np.pi/2, 0)

# X intersection
x12, y12 = get_spline([-1,0], [0,1], 0, np.pi/2)
x13, y13 = get_spline([-1,0], [0,-1], 0, -np.pi/2)
x14, y14 = get_spline([-1,0], [1,0], 0, 0)
x15, y15 = get_spline([0,-1], [1,0], np.pi/2, 0)
x16, y16 = get_spline([0,1], [0,-1], -np.pi/2, -np.pi/2)
x17, y17 = get_spline([0,1], [1,0], -np.pi/2, 0)

# Y intersection
x18, y18 = get_spline([5.5,0], [6.5, 0.5], 0, np.pi/4)
x19, y19 = get_spline([5.5,0], [6.5,-0.5], 0, -np.pi/4)
x20, y20 = get_spline([6.5,0.5], [6.5,-0.5], -(np.pi/4+np.pi/2), -np.pi/4)

# vehicle trajectories
# car 1 path
x_car_1 = np.hstack((x3, x14, x6, x18, x8))
y_car_1 = np.hstack((y3, y14, y6, y18, y8))   

# car 2 path 
x_car_2 = np.hstack((x7[::-1], x19[::-1], x6[::-1], x14[::-1], x3[::-1]))
y_car_2 = np.hstack((y7[::-1], y19[::-1], y6[::-1], y14[::-1], y3[::-1]))

# car 3 path
x_car_3 = np.hstack((x4, x17, x6, x18, x8))
y_car_3 = np.hstack((y4, y17, y6, y18, y8))

# car 4 path
x_car_4 = np.hstack((x1[::-1], x9, x3, x12, x4[::-1]))
y_car_4 = np.hstack((y1[::-1], y9, y3, y12, y4[::-1]))

# car 5 path
x_car_5 = np.hstack((x5[::-1], x13[::-1], x3[::-1], x11[::-1], x2[::-1]))
y_car_5 = np.hstack((y5[::-1], y13[::-1], y3[::-1], y11[::-1], y2[::-1]))

# converting car path into geometry_msgs/Point
car_1_route = []
for i in range(len(x_car_1)):
    car_1_route.append(Point(x_car_1[i], y_car_1[i], 0))

car_2_route = []
for i in range(len(x_car_2)):
    car_2_route.append(Point(x_car_2[i], y_car_2[i], 0))

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

# plt.show()