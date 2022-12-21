import numpy as np
import matplotlib.pyplot as plt

width = 2

def get_straight(x0, y0, x1, y1, theta, steps=100):
    x = np.linspace(x0, x1, steps)
    y = np.linspace(y0, y1, steps)

    # horizontal road
    if theta == 0:
        y_boundary_1 = [a-(width/2) for a in y]
        y_boundary_2 = [a+(width/2) for a in y]
        return x, y, y_boundary_1, y_boundary_2
    # vertical road
    else:
        x_boundary_1 = [a-(width/2) for a in x]
        x_boundary_2 = [a+(width/2) for a in x]
        return x, y, x_boundary_1, x_boundary_2

def get_spline(x0, x1, y0, y1, theta0, theta1, steps=100):
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

	X = np.asarray([x0, dx0, x1, dx1]).transpose()
	bx = np.linalg.solve(Ax, X)

	Ay = np.asarray([[1, t0,   t0**2,   t0**3],  # x  @ 0
					[0, 1,  2*t0,    3*t0**2],  # x' @ 0
					[1, t1,   t1**2,   t1**3],  # x  @ 1
					[0, 1,  2*t1,    3*t1**2]]) # x' @ 1
	Y = np.asarray([y0, dy0, y1, dy1]).transpose()
	by = np.linalg.solve(Ay, Y)

	x = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(),bx)
	y = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(),by)

	return x, y

def register_lane(x, y,count):
    lane_center = [x, y]
    id = count
    return lane_center, id

if __name__ == '__main__':
    x1, y1, bound1_1, bound1_2 = get_straight(-5,0,5,0,0)            # horizontal road 
    x2, y2, bound2_1, bound2_2 = get_straight(6,1,6,11,np.pi/2)      # vertical road
    x3, y3, bound3_1, bound3_2 = get_straight(7,0,17,0,0)           # horizontal road
    x4, y4, bound4_1, bound4_2 = get_straight(6,-1,6,-11,-np.pi/2)    # vertical road

    # lanes
    count = 0
    lane_center1, id1 = register_lane(x1, y1, count)
    lane_center2, id2 = register_lane(x2, y2, count+1)
    lane_center3, id3 = register_lane(x3, y3, count+2)
    lane_center4, id4 = register_lane(x4, y4, count+4)

    # plotting
    plt.plot(x1, y1, 'b--', markersize=1)
    plt.plot(x1, bound1_1, 'k')
    plt.plot(x1, bound1_2, 'k')
    plt.plot(x2, y2, 'b--', markersize=1)
    plt.plot(bound2_1, y2, 'k')
    plt.plot(bound2_2, y2, 'k')
    plt.plot(x3, y3, 'b--', markersize=1)
    plt.plot(x3, bound3_1, 'k')
    plt.plot(x3, bound3_2, 'k')
    plt.plot(x4, y4, 'b--', markersize=1)
    plt.plot(bound4_1, y4, 'k')
    plt.plot(bound4_2, y4, 'k')
    # x, y = get_spline(0,3,0,3,np.pi/2,np.pi/9)
    # x1,y1 = get_spline(3,6,3,0,np.pi/9,np.pi/5)
    # plt.plot(x,y)
    # plt.plot(x1,y1)
    plt.show()