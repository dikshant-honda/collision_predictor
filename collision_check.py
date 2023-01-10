import math
import numpy as np
import matplotlib.pyplot as plt
from environment import *
from lane_publisher import *
from traffic_msgs import *
from nav_msgs import *
from geometry_msgs import *
from update_pos import *

'''
TO DO:
1. change the line intersection function because:
a. line intersection isn't the best way to check the collision because it will not take
   into account the vehicles which are moving across the junction (check memo).
b. it will also not work when the vehicle future trajectories are in spline form
2. improve the initial few lines of line intersection function, redundantly define
3. update the current_waypoint in geometry_msgs.pose.position
4. make a separate class or dictionary for vehicle dynamic data like its pos and vel
5. check the usage of interp_back_path, what if the vehicle doesn't want to follow the
   center lane. 
6. figure out a better way to initialize the future_traj variable
7. again maintain a class for the future trajectory of the vehicles, maybe a maintain a
   proper class for each vehicle dynamics
'''

def lineIntersection(traj_1, traj_2):
    # check the intersection of the future trajectory of the vehicles
    p1_start = Point2D(traj_1[0][0], traj_1[1][0])
    p1_end = Point2D(traj_1[0][-1], traj_1[1][-1])
    p2_start = Point2D(traj_2[0][0], traj_2[1][0])
    p2_end = Point2D(traj_2[0][-1], traj_2[1][-1])
    intersect = Point2D(0,0)
    p0_x = p1_start.x
    p0_y = p1_start.y
    p1_x = p1_end.x
    p1_y = p1_end.y
    p2_x = p2_start.x
    p2_y = p2_start.y
    p3_x = p2_end.x
    p3_y = p2_end.y

    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y))/(-s2_x * s1_y + s1_x * s2_y)
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x))/(-s2_x * s1_y + s1_x * s2_y)

    if s >= 0 and s <= 1 and t >=0 and t <= 1:
        # collision detected
        intersect.x = p0_x + (t * s1_x)
        intersect.y = p0_y + (t * s1_y)
        print("!!COLLISION!!")
        return True
    
    return False  # no collision

def get_future_trajectory(x, y, current_waypoint, v):
    # return the future trajectory of the vehicle
    lane_line_list, lane_s_map = get_lane_and_s_map(x, y)
    future_x, future_y = PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map, v)
    current_waypoint = move(current_waypoint[0], current_waypoint[1], v, dt_m, lane_line_list)
    # update these waypoints as ros messages -> geometry_msgs.pose.position
    # later provide this information on ros traffic messages

    return [future_x, future_y], current_waypoint

# main function
if __name__ == '__main__':
    interp_back_path = 5
    plan_t_m = 1
    dt_m = 0.1
    np_m = int(plan_t_m/dt_m)

    # path
    x, y = get_spline(0,3,0,3,np.pi/2,np.pi/4)
    x_,y_ = get_spline(3,6,3,0,np.pi/2,np.pi/4)
    x1 = x + x_
    y1 = y + y_
    plt.plot(x1, y1, 'g')

    x, y = get_spline(0,3,0,3,-np.pi/2,-np.pi/4)
    x_,y_ = get_spline(3,6,3,0,-np.pi/2,-np.pi/4)
    x2 = x + x_
    y2 = y + y_
    plt.plot(x2, y2, 'k')

    # velocity obtained from vehicles.twist.twist.linear.x
    v_1 = 0.9                   # change later
    v_2 = 0.5

    current_waypoint_1 = [3.04, 3.05]
    current_waypoint_2 = [4.5, 2.8]

    # initialization of future_trajectories as their starting point
    future_traj_1 = [[current_waypoint_1[0], current_waypoint_1[1]], [current_waypoint_1[0]+0.1, current_waypoint_1[1]+0.1]]
    future_traj_2 = [[current_waypoint_2[0], current_waypoint_2[1]], [current_waypoint_2[0]+0.1, current_waypoint_2[1]+0.1]]

    while not lineIntersection(future_traj_1, future_traj_2):
        future_traj_1, current_waypoint_1 = get_future_trajectory(x1, y1, current_waypoint_1,v_1)
        future_traj_2, current_waypoint_2 = get_future_trajectory(x2, y2, current_waypoint_2, v_2)
        plt.plot(future_traj_1[0], future_traj_1[1], 'r--')
        plt.plot(current_waypoint_1[0], current_waypoint_1[1], 'b*')
        plt.plot(future_traj_2[0], future_traj_2[1], 'g--')
        plt.plot(current_waypoint_2[0], current_waypoint_2[1], 'b*')
        plt.pause(0.2)
    plt.show()