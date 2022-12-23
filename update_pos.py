import math
import numpy as np
import matplotlib.pyplot as plt
from environment import *
from lane_publisher import *
from traffic_msgs import *
from nav_msgs import *
from geometry_msgs import *

def get_lane_and_s_map(x1, y1):
    pose_arr = []
    # x_g, y_g = get_spline(current_waypoint[0],destination_waypoint[0],current_waypoint[1],destination_waypoint[1],np.pi/2,np.pi/4)
    lane_route = []
    for i in range(len(x1)):
        lane_route.append([x1[i], y1[i]])
    
    for i in range(len(lane_route)-1):
        point = Point(lane_route[i][0], lane_route[i][1])
        # replace this by actual yaw of the vehicle maybe
        yaw = math.atan2((lane_route[i+1][1]-lane_route[i][1]),(lane_route[i+1][0]-lane_route[i][0]))
        out = quaternion_from_euler(0,0,yaw)
        quat = Quaternion(out[0], out[1], out[2], out[3])
        poses = PoseStamped(std_msgs.Header(), Pose(point, quat))
        pose_arr.append(poses)
    # adding the last point
    pose_arr.append(PoseStamped(std_msgs.Header(), Pose(Point(lane_route[-1][0], lane_route[-1][1]), Quaternion(out[0], out[1], out[2], out[3]))))
    path_route = Path(std_msgs.Header(), pose_arr)
    lane_line_list, lane_s_map = path_to_list(path_route)

    return lane_line_list, lane_s_map

def PredictTrajectoryVehicles(init_x, init_y, path, s_map):    # msg_vehicles -> traffic_msg/PredictionArray
    s, d, _ = get_frenet(init_x, init_y, path, s_map)
    future_x = []
    future_y = []
    for t in range(np_m):
        if t < interp_back_path:
            d_val = d - ((t*d) / interp_back_path)
            new_x, new_y, _ = get_xy(s+v*dt_m*t, d_val, path, s_map)
        else:
            new_x, new_y, _ = get_xy(s+v*dt_m*t, 0, path, s_map)
        future_x.append(new_x)
        future_y.append(new_y)
    return future_x, future_y

def move(x, y, v, dt_m, path):
    # find the closest index from the curve and compute theta between those points
    # shift the vehicle along that direction to get the modified points
    # in real world, we will get it from our detection algorithm

    ind_closest = closest_point_ind(path, x, y)
    # Determine the indices of the 2 closest points
    if ind_closest < len(path):
        # Check if we are at the end of the segment
        if ind_closest == len(path) - 1:
            use_previous = True
        elif ind_closest == 0:
            use_previous = False
        else:
            dist_prev = distance(path[ind_closest-1].x, path[ind_closest-1].y, x, y)
            dist_next = distance(path[ind_closest+1].x, path[ind_closest+1].y, x, y)

            if dist_prev <= dist_next:
                use_previous = True
            else:
                use_previous = False

        # Get the 2 points
        if use_previous:
            p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
            p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)

        # Get the point in the local coordinate with center p1
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
    new_x = x + v*np.cos(theta)*dt_m
    new_y = y + v*np.sin(theta)*dt_m

    return [new_x, new_y]

# main function
if __name__ == '__main__':
    interp_back_path = 5
    plan_t_m = 1
    dt_m = 0.1
    np_m = int(plan_t_m/dt_m)

    # path
    x, y = get_spline(0,3,0,3,np.pi/2,np.pi/4)
    x_,y_ = get_spline(3,6,3,0,np.pi/2,np.pi/4)
    x = x + x_
    y = y + y_
    plt.plot(x, y)

    # velocity obtained from vehicles.twist.twist.linear.x
    v = 0.5                   # change later

    current_waypoint = [3.04, 3.05]

    # replace by collision check
    horizon = 0
    while horizon < 100:
        lane_line_list, lane_s_map = get_lane_and_s_map(x, y)
        future_x, future_y = PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map)
        current_waypoint = move(current_waypoint[0], current_waypoint[1], v, dt_m, lane_line_list)
        horizon += 1
        plt.plot(future_x, future_y, 'r--')
        plt.plot(current_waypoint[0], current_waypoint[1], 'b*')
        plt.pause(0.2)
    plt.show()