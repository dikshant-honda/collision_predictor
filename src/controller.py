#! /usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from frenet import *
from pid_planner import PI

class Controller:
    def __init__(self):
        pass

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
    def get_lane_and_s_map(self, route):
        x, y = [], []
        for i in  range(len(route)):
            x.append(route[i].x)
            y.append(route[i].y)
        
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
        path_route = Path(Header, pose_arr)
        lane_line_list, lane_s_map = self.path_to_list(path_route)

        return lane_line_list, lane_s_map
    
    def closest_point_ind(self, path, x, y):
        # finding the closest index on path from point(x,y)
        index = 0
        closest_index = 0
        min_dist = 10000.0
        for p in path:
            dist = distance(p.x, p.y, x, y)
            if dist < min_dist:
                min_dist = dist
                closest_index = index
            index += 1
        return closest_index

    def correct_angle(self, angle):
        if angle > np.pi:
            angle = angle - 2*np.pi
        elif angle < -np.pi:
            angle = angle + 2*np.pi
        
        return angle

    def step(self, car):
        path, _  = self.get_lane_and_s_map(car.car_route)
        x_pos, y_pos = car.pose.pose.pose.position.x, car.pose.pose.pose.position.y
        ind_closest = self.closest_point_ind(path, x_pos, y_pos)        
        yaw_path = car.car_yaw
        # still on the lane
        if ind_closest < len(path)-1:
            x, y, z, w = car.pose.pose.pose.orientation.x, car.pose.pose.pose.orientation.y, car.pose.pose.pose.orientation.z, car.pose.pose.pose.orientation.w
            _, _, init_yaw = euler_from_quaternion([x, y, z, w])

            # PI controller for yaw correction
            pi = PI(P=7.7, I = 10.0)
            yaw_desired = yaw_path[ind_closest]
            feedback = self.correct_angle(init_yaw)
            ang_error = yaw_desired - feedback
            ang_error = self.correct_angle(ang_error)
            pi.update(-ang_error)
            omega = pi.output
            
            v = car.vel 

            # twist message to be published
            linear = Vector3(v, 0, 0)
            angular = Vector3(0, 0, omega)
            move = Twist(linear, angular)
            car.stop = False

        # stop after reaching the end of lane
        else:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)
            move = Twist(linear, angular)

        return car, move