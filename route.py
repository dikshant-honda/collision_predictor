import numpy as np
import matplotlib.pyplot as plt
from environment import *
from lane_publisher import *
from traffic_msgs import *
from nav_msgs import *
from geometry_msgs import *

x1, y1, bound1_1, bound1_2 = get_straight(-5,0,5,0,0)            # horizontal road 
x2, y2, bound2_1, bound2_2 = get_straight(6,1,6,11,np.pi/2)      # vertical road
x3, y3, bound3_1, bound3_2 = get_straight(7,0,17,0,0)           # horizontal road
x4, y4, bound4_1, bound4_2 = get_straight(6,-1,6,-11,-np.pi/2)    # vertical road

count = 0
lane_center1, id1 = register_lane(x1, y1, count)
lane_center2, id2 = register_lane(x2, y2, count+1)
lane_center3, id3 = register_lane(x3, y3, count+2)
lane_center4, id4 = register_lane(x4, y4, count+4)

# figure out
# lane_info = lane_publisher.publish_all_lanes()  # get lanes info without carla

current_waypoint = [-2,0]
destination_waypoint = [3,0]
lane_route = np.linspace(current_waypoint, destination_waypoint, 10)   # replace this by a path planning algorithm to get the modified route
pt = Point(current_waypoint[0], current_waypoint[1])
quat = Quaternion()
pose = Pose(pt,quat)
head = std_msgs.Header()
poses = PoseStamped(head,pose)
current_pose = poses
# print(current_pose.pose.position.x)

pose_arr = [] 
for i in range(len(lane_route)):
    pt = Point(lane_route[i][0], lane_route[i][1])
    quat = Quaternion()
    pose = Pose(pt, quat)
    poses = PoseStamped(std_msgs.Header(), pose)
    pose_arr.append(poses)

path_route = Path(std_msgs.Header(), pose_arr)
prev_wp = None
print(path_route)