import math
import numpy as np
import matplotlib.pyplot as plt
from environment import *
from lane_publisher import *
from traffic_msgs import *
from nav_msgs import *
from geometry_msgs import *
from update_pos import *

def lineIntersection(traj_1, traj_2):
    p1_start = Point2D(traj_1[0].x, traj_1[0].y)
    p1_end = Point2D(traj_1[-1].x, traj_1[-1].y)
    p2_start = Point2D(traj_2[0].x, traj_2[0].y)
    p2_end = Point2D(traj_2[-1].x, traj_2[-1].y)
    intersect = Point2D()
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
        return True
    
    return False  # no collision

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
    v = 0.5                   # change later

    current_waypoint_1 = [3.04, 3.05]
    current_waypoint_2 = [4.5, 2.08]

    # initialization of future_trajectories as their starting point
    future_traj_1 = current_waypoint_1
    future_traj_2 = current_waypoint_2

    while lineIntersection(future_traj_1, future_traj_2):
        lane_line_list, lane_s_map = get_lane_and_s_map(x, y)
        future_x, future_y = PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map)
        current_waypoint = move(current_waypoint[0], current_waypoint[1], v, dt_m, lane_line_list)
        # update these waypoints as ros messages -> geometry_msgs.pose.position
        # later provide this information on ros traffic messages
        horizon += 1
        plt.plot(future_x, future_y, 'r--')
        plt.plot(current_waypoint[0], current_waypoint[1], 'b*')
        plt.pause(0.2)
    plt.show()