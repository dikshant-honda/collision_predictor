#! /usr/bin/env python3

import math
import itertools
from helper.frenet import *
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

class Predictions:
    def __init__(self, env, lanes):
        # variables
        self.interp_back_path = 1000                    # interpolate back to path after this # of steps
        self.plan_t_m = 5                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 1.4                                  # tolerance value for proximity check
        self.vision_radius = 3                          # check only nearby cars
        self.env = env
        self.lanes = lanes

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
        # adding the last point
        pose_arr.append(PoseStamped(Header, Pose(Point2D(lane_route[-1][0], lane_route[-1][1]), quat)))
        path_route = Path(Header, pose_arr)
        lane_line_list, lane_s_map = path_to_list(path_route)

        return lane_line_list, lane_s_map
    
    # future waypoints
    def PredictTrajectoryVehicles(self, init_x, init_y, path, s_map, v, d):   
        s, d_curr, _, _ = get_frenet_with_theta(init_x, init_y, path, s_map)
        d = (d_curr + d) / 2                    # average of all the deviations from center
        future_waypoints = []
        for t in range(self.np_m):
            if t < self.interp_back_path:
                d_val = d - ((t*d) / self.interp_back_path)
                new_x, new_y, _ = get_xy(s+v*self.dt_m*t, d_val, path, s_map)
            else:
                new_x, new_y, _ = get_xy(s+v*self.dt_m*t, 0, path, s_map)
            future_waypoints.append(Point(new_x, new_y, 0.0))
        return future_waypoints, d

    # getting the future trajectory
    def get_future_trajectory(self, car): 
        v = car.vel
        d = 0
        lane_line_list, lane_s_map = self.get_lane_and_s_map(car.car_route)
        future_waypoints, d = self.PredictTrajectoryVehicles(car.pose.pose.pose.position.x, car.pose.pose.pose.position.y, lane_line_list, lane_s_map, v, d)
        return future_waypoints

    # collision check by vicinity or point-wise check
    def collision(self, points1, points2):
        for i in range(len(points1)):
            for j in range(len(points2)):
                if distance(points1[i].x, points1[i].y, points2[j].x, points2[j].y) < self.tol:
                    return True
        return False

    def inVicinity(self, car1, car2):
        car1_pos = car1.pose.pose.pose.position
        car2_pos = car2.pose.pose.pose.position
        if distance(car1_pos.x, car1_pos.y, car2_pos.x, car2_pos.y) < self.vision_radius:
            # print("start checking the future trajectories between", car1.id, "and", car2.id)
            return True
        return False

    def add(self, car):
        self.env.register = True
        self.env.vehicles += 1
        self.env.vehicle_states.append(car)

    def removal(self, car):
        if car in self.env.vehicle_states:
            self.env.deregister = True
            self.env.vehicles -= 1
            self.env.vehicle_states.remove(car)

    def EOL(self, car):                         # vehicle has reached the goal point
        self.removal(car)

    def closest_pt_idx(self, x, y, lane):
        # finding the closest index on lane from point(x,y)
        closest_index = 0
        min_dist = 10000.0
        for i in range(len(lane[0])):
            dist = distance(lane[0][i].x, lane[0][i].y, x, y)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        return closest_index
    
    def get_route(self, pos, original_lane, next_lane):
        lane = self.lanes.stack_lanes(original_lane, next_lane)
        idx = self.closest_pt_idx(pos.x, pos.y, lane)
        car_route_ = []
        yaw_route_ = []
        horizon = 0
        while idx < len(lane[0]) and horizon < 500:
            car_route_.append(Point(lane[0][idx].x, lane[0][idx].y, 0))
            yaw_route_.append(lane[1][idx])
            horizon += 1
            idx += 1
        return car_route_, yaw_route_

    def update(self, car):
        possible_lanes = self.lanes.get_turning_routes(car.location)
        for lane in possible_lanes:
            route, _ = self.get_route(car.pose.pose.pose.position, car.location, lane)
            car.car_route_ = route
            car.car_yaw = None

            car.future_waypoints = self.get_future_trajectory(car)

    def predict_collision(self):
        for first_car, second_car in itertools.combinations(self.env.vehicle_states, 2):
            # add the future waypoints here
            if self.collision(first_car.future_waypoints, second_car.future_waypoints):
                print("collision between", first_car.id, "and", second_car.id)