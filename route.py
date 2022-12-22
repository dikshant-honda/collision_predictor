import numpy as np
import matplotlib.pyplot as plt
from environment import *
from lane_publisher import *
from traffic_msgs import *
from nav_msgs import *
from geometry_msgs import *

def frenet_to_xy_test():
    # return cartesian coordinates from frenet coordinates
    pass

def xy_to_frenet():
    # return frenet coordinates from catersian coordinates
    pass

def get_sd(x, y):
    pose_arr = [] 
    # current_waypoint = [-2,0]
    # destination_waypoint = [3,0]
    lane_route = np.linspace([x[0], y[0]], [x[-1], y[-1]], 100)   # replace this by a path planning algorithm to get the modified route
    for i in range(len(lane_route)-1):
        point = Point(lane_route[i][0], lane_route[i][1])
        # replace this by actual yaw of the vehicle maybe
        yaw = np.arctan((lane_route[i+1][1]-lane_route[i][1])/(lane_route[i+1][0]-lane_route[i][0]))
        out = quaternion_from_euler(0,0,yaw)
        quat = Quaternion(out[0], out[1], out[2], out[3])
        poses = PoseStamped(std_msgs.Header(), Pose(point, quat))
        pose_arr.append(poses)
    # adding the last point
    pose_arr.append(PoseStamped(std_msgs.Header(), Pose(Point(lane_route[-1][0], lane_route[-1][1]), Quaternion(out[0], out[1], out[2], out[3]))))
    path_route = Path(std_msgs.Header(), pose_arr)
    lane_line_list, lane_s_map = path_to_list(path_route)

    s_arr = []
    d_arr = []
    for i in range(len(path_route.poses)):
        s, d, _ = get_frenet(path_route.poses[i].pose.position.x, path_route.poses[i].pose.position.y, lane_line_list, lane_s_map) 
        s_arr.append(s)
        d_arr.append(d)

    x_arr = []
    y_arr = []
    for i in range(len(s_arr)):
        x_, y_, _ = get_xy(s_arr[i], d_arr[i], lane_line_list, lane_s_map)
        x_arr.append(x_)
        y_arr.append(y_)

    return s_arr, d_arr, x_arr, y_arr

x1, y1, bound1_1, bound1_2 = get_straight(-5,0,5,0,0)            # horizontal road 
x2, y2, bound2_1, bound2_2 = get_straight(6,1,6,11,np.pi/2)      # vertical road
x3, y3, bound3_1, bound3_2 = get_straight(7,0,17,0,0)           # horizontal road
x4, y4, bound4_1, bound4_2 = get_straight(6,-1,6,-11,-np.pi/2)    # vertical road

count = 0
lane_center1, id1 = register_lane(x1, y1, count)
lane_center2, id2 = register_lane(x2, y2, count+1)
lane_center3, id3 = register_lane(x3, y3, count+2)
lane_center4, id4 = register_lane(x4, y4, count+3)

# figure out
# lane_info = lane_publisher.publish_all_lanes()  # get lanes info without carla

# current_waypoint = [-2,0]
# destination_waypoint = [3,0]
# lane_route = np.linspace(current_waypoint, destination_waypoint, 10)   # replace this by a path planning algorithm to get the modified route

# pose_arr = [] 
# for i in range(len(lane_route)):
#     # change the yaw angle for obtaining different quaternions and then append it to pose
#     poses = PoseStamped(std_msgs.Header(), Pose(Point(lane_route[i][0], lane_route[i][1]), Quaternion()))
#     pose_arr.append(poses)

# path_route = Path(std_msgs.Header(), pose_arr)
# lane_line_list, lane_s_map = path_to_list(path_route)

# s_arr = []
# d_arr = []
# for i in range(len(path_route.poses)):
#     s, d, _ = get_frenet(path_route.poses[i].pose.position.x, path_route.poses[i].pose.position.y, lane_line_list, lane_s_map) 
#     s_arr.append(s)
#     d_arr.append(d)

x, y = get_spline(0,3,0,3,np.pi/2,np.pi/9)
x_,y_ = get_spline(3,6,3,0,np.pi/9,np.pi/5)
x = x + x_
y = y + y_

s_str, d_str, x_str, y_str = get_sd(x1, y1)
s, d, x_org, y_org  = get_sd(x, y)
print(y_str[0])
# plt.plot(x, y)
# plt.plot(d, s, 'b--')
plt.plot(x1, y1)
plt.plot(x_str, y_str)
# plt.plot(d_str, s_str, 'r--')
# plt.plot(x_org, y_org, 'b-')
plt.show()
