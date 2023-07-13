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
        self.interp_back_path = 1000                    # interpolate back to path after this # of steps
        self.plan_t_m = 5                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 1.4                                  # tolerance value for proximity check

    def get_s_map(
            self, 
            route: NDArray,
    ) -> NDArray:                   # update required
        """
        Function to get the s-map in Frenet coordinate system
        It will accumulate the distance along the curve taking origin as the vehicle current position
        """
        s_map = np.array([], dtype = np.float64)
        accumulated_distance = 0.0
        prev_point = None
        for x, y in route:
            if prev_point != None:
                accumulated_distance += distance(prev_point[0], prev_point[1], x, y)
            s_map = np.append(s_map, accumulated_distance)
            prev_point = [x, y]
        return np.array((route, s_map))
  
    # future waypoints
    def PredictTrajectoryVehicles(
            self, 
            init_x: np.float64, 
            init_y: np.float64, 
            path: NDArray, 
            s_map: NDArray[np.float64], 
            v: np.float64, 
            d:np.float64,
        ) -> NDArray:   
        s, d_curr, _, _ = get_frenet_with_theta(init_x, init_y, path, s_map)
        d = (d_curr + d) / 2                    # average of all the deviations from center
        future_waypoints = np.array([], dtype=np.array([], dtype=np.float64))
        for t in range(self.np_m):
            if t < self.interp_back_path:
                d_val = d - ((t*d) / self.interp_back_path)
                new_x, new_y, _ = get_xy(s+v*self.dt_m*t, d_val, path, s_map)
            else:
                new_x, new_y, _ = get_xy(s+v*self.dt_m*t, 0, path, s_map)
            future_waypoints = np.append(future_waypoints, [new_x, new_y])
        return future_waypoints

    # getting the future trajectory
    def get_future_trajectory(self, veh): 
        v = veh.velocity
        d = 0
        lane_line_list, lane_s_map = self.get_s_map(veh.route)
        future_waypoints, d = self.PredictTrajectoryVehicles(veh.pos[0], veh.pose[1], lane_line_list, lane_s_map, v, d)
        return future_waypoints

    # collision check by vicinity or point-wise check
    def collision(self, points1, points2):
        for i in range(len(points1)):
            for j in range(len(points2)):
                if distance(points1[i][0], points1[i][1], points2[j][0], points2[j][1]) < self.tol:
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