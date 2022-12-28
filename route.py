import math
import numpy as np
import matplotlib.pyplot as plt
from environment import *
from lane_publisher import *
from traffic_msgs import *
from nav_msgs import *
from geometry_msgs import *

'''
this script is used for testing the conversion from cartesian to frenet to cartesian
'''
def xy_to_frenet(x1, y1): #, current_waypoint, destination_waypoint):
    # return frenet coordinates from cartesian coordinates
    pose_arr = []
    # x_g, y_g = get_spline(current_waypoint[0],destination_waypoint[0],current_waypoint[1],destination_waypoint[1],np.pi/2,np.pi/4)
    lane_route = []
    for i in range(len(x1)):
        lane_route.append([x1[i], y1[i]])
    
    for i in range(len(lane_route)-1):
        point = Point(lane_route[i][0], lane_route[i][1])
        # replace this by actual yaw of the vehicle maybe
        yaw = math.atan2((lane_route[i+1][1]-lane_route[i][1]),(lane_route[i+1][0]-lane_route[i][0]))
        # print(yaw)
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

    return s_arr, d_arr, lane_line_list, lane_s_map

def frenet_to_xy_test(s_arr, d_arr, lane_line_list, lane_s_map):
    # return cartesian coordinates from frenet coordinates
    x_arr = []
    y_arr = []
    for i in range(1,len(s_arr)):
        x, y, _ = get_xy(s_arr[i], d_arr[i], lane_line_list, lane_s_map)
        x_arr.append(x)
        y_arr.append(y)
    
    return x_arr, y_arr

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

x, y = get_spline(0,3,0,3,np.pi/2,np.pi/4)
x_,y_ = get_spline(3,6,3,0,np.pi/2,np.pi/4)
x = x + x_
y = y + y_

current_waypoint = [3.04, 3.05]
destination_waypoint = [8.45, 2.875]
s, d, lane_line, s_map = xy_to_frenet(x, y)#, current_waypoint, destination_waypoint)
x_test, y_test = frenet_to_xy_test(s, d, lane_line, s_map)
plt.plot(x, y)
plt.plot(d, s)
plt.plot(x_test, y_test, 'r*')
plt.show()
