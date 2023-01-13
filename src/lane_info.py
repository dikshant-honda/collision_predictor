import numpy as np 
import matplotlib.pyplot as plt

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

if __name__ == '__main__':
    #  straight lanes
    x1, y1 = get_lanes([-6,-1],[-6,-8])     # down right
    x2, y2 = get_lanes([-6,8], [-6,1])      # down left
    x3, y3 = get_lanes([-1,0], [-5,0])      # down center
    x4, y4 = get_lanes([0,5], [0,1])        # center left
    x5, y5 = get_lanes([0,-1], [0,-5])       # center right
    x6, y6 = get_lanes([1,0], [5.5,0])      # up center
    x7, y7 = get_lanes([6.5,-0.5], [10,-4]) # up right
    x8, y8 = get_lanes([6.5,0.5], [10,4])   # up left

    # splines
    x9, y9 = get_spline([-6,-1], [-5,0], np.pi/2,0)
    x10, y10 = get_spline([-6,-1], [-6,1], np.pi/2, -np.pi/2)
    x11, y11 = get_spline([-6,1], [-5,0], -np.pi/2, 0)
    plt.plot(x1, y1, 'k')
    plt.plot(x2, y2, 'k')  
    plt.plot(x3, y3, 'k')  
    plt.plot(x4, y4, 'k')  
    plt.plot(x5, y5, 'k')  
    plt.plot(x6, y6, 'k')  
    plt.plot(x7, y7, 'k')  
    plt.plot(x8, y8, 'k')

    plt.plot(x9, y9, 'b--')  
    plt.plot(x10, y10, 'b--')  
    plt.plot(x11, y11, 'b--')

    plt.show()