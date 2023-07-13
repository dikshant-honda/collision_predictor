#! /usr/bin/env python3

from dataclasses import dataclass
import itertools
import numpy as np
from numpy.typing import NDArray
from helper.frenet import *

@dataclass(frozen=True)
class Predictions:
    interp_back_path: int
    plan_t_m: int
    dt_m: float
    np_m: float
    tol: float

    def __init__(self) -> None:
        # variables
        self.interp_back_path = 1000                    # interpolate back to path after this number of steps
        self.plan_t_m = 5                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 1.4                                  # tolerance value for proximity check

    def get_s_map(
            self, 
            path: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """
        Function to get the s-map in Frenet coordinate system
        It will accumulate the distance along the curve taking origin as the vehicle current position
        args: path: 2D array having the coordinates of the middle lane of the road 
        """
        s_map = np.array([], dtype = np.float64)
        accumulated_distance = 0.0
        prev_point = None
        for x, y in path:
            if prev_point != None:
                accumulated_distance += distance(prev_point[0], prev_point[1], x, y)
            s_map = np.append(s_map, accumulated_distance)
            prev_point = [x, y]
        return s_map
  
    def PredictTrajectoryVehicles(
            self, 
            init_x: np.float64, 
            init_y: np.float64, 
            path: NDArray, 
            s_map: NDArray[np.float64], 
            velocity: np.float64, 
            d:np.float64,
        ) -> NDArray[np.float64]:
        """
        Function to predict the future trajectory of the vehicle along the lanes

        args:
            init_x, init_y: current x,y coordinates of the car
            path: middle lane coordinates
            s_map: cumulative distance along the lanes
            velocity: measured velocity of the vehicle from the perception 
            d: offset from the center lane
        """
        s, d_curr, _, _ = get_frenet_with_theta(init_x, init_y, path, s_map)
        d = (d_curr + d) / 2      # average of all the deviations from center
        future_waypoints_x = np.array([], dtype = np.float64)
        future_waypoints_y = np.array([], dtype = np.float64)
        for t in range(self.np_m):
            if t < self.interp_back_path:
                d_val = d - ((t*d) / self.interp_back_path)
                new_x, new_y, _ = get_xy(s+velocity*self.dt_m*t, d_val, path, s_map)
            else:
                new_x, new_y, _ = get_xy(s+velocity*self.dt_m*t, 0, path, s_map)
            future_waypoints_x = np.append(future_waypoints_x, new_x)
            future_waypoints_y = np.append(future_waypoints_y, new_y)
        return np.array((future_waypoints_x, future_waypoints_y))

    # getting the future trajectory
    def get_future_trajectory(self, veh): 
        v = veh.velocity
        d = 0
        lane_s_map = self.get_s_map(veh.route)
        future_waypoints = self.PredictTrajectoryVehicles(veh.pos[0], veh.pose[1], veh.route, lane_s_map, v, d)
        return future_waypoints

    # collision check by vicinity or point-wise check
    def collision(self, points1, points2):
        for i in range(len(points1)):
            for j in range(len(points2)):
                if distance(points1[0][i], points1[1][i], points2[0][j], points2[1][j]) < self.tol:
                    return True
        return False

    def add(self, car, env):
        env.register = True
        env.counter += 1
        env.vehicles.append(car)

    def removal(self, car, env):
        if car in self.env.vehicles:
            env.deregister = True
            env.counter -= 1
            env.vehicles.remove(car)

    def closest_pt_idx(self, x, y, lane):
        # finding the closest index on lane from point(x,y)
        closest_index = 0
        min_dist = 10000.0
        for i in range(len(lane[0])):
            dist = distance(lane[0][i][0], lane[0][i][1], x, y)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        return closest_index
    
    def get_route(self, pos, lanes, curr_lane, next_lane):
        lane = lanes.stack_lanes(curr_lane, next_lane)
        idx = self.closest_pt_idx(pos[0], pos[1], lane)
        route = []
        horizon = 0
        while idx < len(lane[0]) and horizon < 500:
            route.append(lane[0][idx][0], lane[0][idx][1])
            horizon += 1
            idx += 1
        return route
    
    # assign next possible movement and estimate future trajectory 
    def update(self, veh, lanes):
        possible_lanes = lanes.get_turning_routes(veh.location)
        for next_lane in possible_lanes:
            route = self.get_route(veh.pos, lanes, veh.location, lanes, next_lane)
            veh.route = route
            # get future coordinates
            veh.future_waypoints = self.get_future_trajectory(veh)

    # updates the dynamics for this
    def predict_collision(self, env):
        for first_car, second_car in itertools.combinations(env.vehicles, 2):
            if self.collision(first_car.future_waypoints, second_car.future_waypoints):
                print("collision between", first_car.id, "and", second_car.id)