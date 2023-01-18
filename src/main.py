#! /usr/bin/env python3

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler
from frenet import *
from geometry_utils import *
from lane_info import *
from collision_predictor.msg import *

class Subscriber:
    def __init__(self):
        # position subcribers
        self.sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.callback1)
        self.sub2 = rospy.Subscriber('/tb3_2/odom', Odometry, self.callback2)
        self.sub3 = rospy.Subscriber('/tb3_3/odom', Odometry, self.callback3)
        self.sub4 = rospy.Subscriber('/tb3_4/odom', Odometry, self.callback4)
        self.sub5 = rospy.Subscriber('/tb3_5/odom', Odometry, self.callback5)

    # collision check
    def lineIntersection(self, traj_1_x, traj_1_y, traj_2_x, traj_2_y):
        intersect = Point2D(0,0)
        p0_x = traj_1_x[0]
        p0_y = traj_1_y[0]
        p1_x = traj_1_x[-1]
        p1_y = traj_1_y[-1]
        p2_x = traj_2_x[0]
        p2_y = traj_2_y[0]
        p3_x = traj_2_x[-1]
        p3_y = traj_2_y[-1]

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
            print("!!COLLISION AHEAD!!")
            return True
        
        return False  # no collision

    # converting ther nav_path message type to list for ease in accessibility
    # generating s_map from the start point till end point for transforms
    def path_to_list(self, nav_path):
        path_list = []
        distance_acum = 0.0
        s_map = []
        prev_p = None
        for pose in nav_path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            path_list.append(Point2D(x, y))
            if prev_p != None:
                distance_acum += distance(prev_p.x, prev_p.y, x, y)
            s_map.append(distance_acum)
            prev_p = Point2D(x, y)
        return path_list, s_map

    # get the s_map and lane info
    def get_lane_and_s_map(self, x, y):
        pose_arr = []
        lane_route = []
        for i in range(len(x)):
            lane_route.append([x[i], y[i]])
        
        for i in range(len(lane_route)-1):
            point = Point2D(lane_route[i][0], lane_route[i][1])
            yaw = math.atan2((lane_route[i+1][1]-lane_route[i][1]),(lane_route[i+1][0]-lane_route[i][0]))
            quat = quaternion_from_euler(0,0,yaw)
            poses = PoseStamped(Header, Pose(point, quat))
            pose_arr.append(poses)
        # adding the last point
        pose_arr.append(PoseStamped(Header, Pose(Point2D(lane_route[-1][0], lane_route[-1][1]), quat)))
        path_route = Path(Header, pose_arr)
        lane_line_list, lane_s_map = self.path_to_list(path_route)

        return lane_line_list, lane_s_map

    # compute the closest points on the route
    def closest_points(self, ind_closest, path, x, y):
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

        if use_previous:
            p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
            p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p3 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            p3 = Point2D(path[ind_closest + 2].x, path[ind_closest + 2].y)
            prev_idx = ind_closest
        
        return p1, p2, p3, prev_idx

    # adding on theta to get the yaw of the vehicle
    def get_frenet_with_theta(self, x, y, path, s_map):
        if path == None:
            print("Empty map. Cannot return Frenet coordinates")
            return 0.0, 0.0, 0.0

        ind_closest = closest_point_ind(path, x, y)

        # Determine the indices of the 2 closest points
        if ind_closest < len(path):
            p1, p2, _, prev_idx = self.closest_points(ind_closest, path, x, y)
            # Get the point in the local coordinate with center p1
            theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
            local_p = global_to_local(p1, theta, Point2D(x,y))

            # Get the coordinates in the Frenet frame
            p_s = s_map[prev_idx] + local_p.x
            p_d = local_p.y

        else:
            print("Incorrect index")
            return 0.0, 0.0, 0.0

        return p_s, p_d, theta

    # Transform from Frenet s,d coordinates to Cartesian x,y
    def get_xy(self, s, d, path, s_map):

        if path == None or s_map == None:
            print("Empty path. Cannot compute Cartesian coordinates")
            return 0.0, 0.0

        # If the value is out of the actual path send a warning
        if s < 0.0 or s > s_map[-1]:
            if s < 0.0:
                prev_point = 0
            else:
                prev_point = len(s_map) -2
        else:
            # Find the previous point
            idx = bisect.bisect_left(s_map, s)
            prev_point = idx - 1

        p1 = path[prev_point]
        p2 = path[prev_point + 1]

        # Transform from local to global
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
        p_xy = local_to_global(p1, theta, Point2D(s - s_map[prev_point], d))

        return p_xy.x, p_xy.y

    # future waypoints
    def PredictTrajectoryVehicles(self, init_x, init_y, path, s_map, v, d):   
        s, d_curr, _ = self.get_frenet_with_theta(init_x, init_y, path, s_map)
        d = (d_curr + d) / 2                    # average of all the deviations from center
        future_x = []
        future_y = []
        for t in range(self.np_m):
            if t < self.interp_back_path:
                d_val = d - ((t*d) / self.interp_back_path)
                new_x, new_y = self.get_xy(s+v*self.dt_m*t, d_val, path, s_map)
            else:
                new_x, new_y = self.get_xy(s+v*self.dt_m*t, 0, path, s_map)
            future_x.append(new_x)
            future_y.append(new_y)
        return future_x, future_y, d

    # getting the future trajectory
    def get_future_trajectory(self, x, y, current_waypoint, past_v, w, past_d, pub):
        v = np.mean(past_v)
        d = np.mean(past_d)
        lane_line_list, lane_s_map = self.get_lane_and_s_map(x, y)
        future_x, future_y, d = self.PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map, v, d)
        # future trajectory starting from the current waypoint
        future_x.insert(0, current_waypoint[0])
        future_y.insert(0, current_waypoint[1])

        return future_x, future_y, d

    def callback1(self, msg):
        self.car_1_pose = msg.pose.pose

    # moving the car along the route
    def update(self, path, x, y, vel):
        stop = False
        ind_closest = closest_point_ind(path, x, y)
        # still on the lane
        if ind_closest < len(path)-2:
            p1, p2, p3, _ = self.closest_points(ind_closest, path, x, y)

            yaw = math.atan((p2.y - p1.y)/(p2.x - p1.x))
            next_yaw = math.atan((p3.y - p2.y)/ (p3.x - p2.x))

            linear = Vector3(vel*math.cos(yaw), vel*math.sin(yaw), 0)
            angular = Vector3(0, 0, 3*(next_yaw - yaw)/self.dt_m)
            move = Twist(linear, angular)

        # stop after reaching the end of lane
        else:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)
            move = Twist(linear, angular)
            stop = True

        return move, stop

    def main(self):
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    try:
        # initialize the trajectories
        # initialize the vehicles
        car_1 = VehicleState()
        car_2 = VehicleState()
        car_3 = VehicleState()
        car_4 = VehicleState()
        car_5 = VehicleState()

        rospy.init_node('predictor', anonymous=True)
        pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
        pub4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=10)
        pub5 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=10)
        sub = Subscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass