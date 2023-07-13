#! /usr/bin/env python3

from dataclasses import dataclass
import itertools
import numpy as np
from numpy.typing import NDArray
from helper.frenet import *
from env_info.vehicle_info import Traffic
from env_info.environment import Environment
from env_info.lane_info import LaneInfo

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
        Function to get the s-map in Frenet coordinate system.
        It will accumulate the distance along the curve taking origin as the vehicle current position.
        
        args: 
            path: 2D array having the coordinates of the middle lane of the road 
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
            position: NDArray[np.float64],
            path: NDArray, 
            s_map: NDArray[np.float64], 
            velocity: float, 
            d:float,
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
        init_x, init_y = position[0], position[1]
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

    def get_future_trajectory(
            self, 
            vehicle: Traffic,
        ) -> NDArray[np.float64]:
        """
        Function to obtain the future trajectory of the vehicle
        
        args:
            vehicle: traffic type vehicle carrying all the relevant information 
        """
        position = vehicle.position
        path = vehicle.possible_route 
        velocity = vehicle.velocity
        d = 0
        s_map = self.get_s_map(path)
        future_waypoints = self.PredictTrajectoryVehicles(position, path, s_map, velocity, d)
        return future_waypoints

    # reduce the computational cost for this
    def closest_pt_idx(
            self, 
            x: float, 
            y: float, 
            lane: NDArray[np.float64],
        ) -> float:
        """
        Function to find the closest index on the lane to the current position of the traffic agent
        
        args:
            x,y: current position of the vehicle
            lane: current lane on which the traffic agent is moving
        """
        closest_index = 0
        min_dist = 10000.0
        for i in range(len(lane[0])):
            dist = distance(lane[0][i][0], lane[0][i][1], x, y)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        return closest_index
    
    def get_route(
            self, 
            position: NDArray[np.float64], 
            lanes: LaneInfo, 
            current_lane: LaneInfo, 
            next_lane: LaneInfo,
            horizon: int = 500,
        ) -> LaneInfo:
        """
        Function to stack the next lane with the current lane on the intersection.
        
        args:
            position: current position of the vehicle
            lanes: defining the lanes type
            current_lane: lane at which car is located currently
            next_lane: next lane to stack with the current lane
            horizon: number of points to add into the path
        """
        lane = lanes.stack_lanes(current_lane, next_lane)
        idx = self.closest_pt_idx(position[0], position[1], lane)
        route = []
        horizon = 0
        while idx < len(lane[0]) and horizon:
            route.append(lane[0][idx][0], lane[0][idx][1])
            horizon += 1
            idx += 1
        return route
    
    def update(
            self, 
            vehicle: Traffic, 
            lanes: LaneInfo,
            at_intersection: bool,
        ) -> None:
        """
        Function to update the dynamics of the traffic agent. 
        Assign new lane and future waypoints to the traffic agent.

        args:
            vehicle: traffic agent whose dynamics need to be updated
            lanes: current lane and switching possibilty if it arrives at intersecton
        """
        if at_intersection:
            possible_lanes = lanes.get_turning_routes(vehicle.location)
            for next_lane in possible_lanes:
                route = self.get_route(vehicle.position, lanes, vehicle.location, next_lane)
                vehicle.location = route
                vehicle.future_waypoints = self.get_future_trajectory(vehicle)
        else:
            route = self.get_route(vehicle.position, lanes, vehicle.location, [])
            vehicle.location = route
            vehicle.future_waypoints = self.get_future_trajectory(vehicle)

    # collision check by vicinity or point-wise check
    def collision(self, points1, points2):
        for i in range(len(points1)):
            for j in range(len(points2)):
                if distance(points1[0][i], points1[1][i], points2[0][j], points2[1][j]) < self.tol:
                    return True
        return False

    # updates the dynamics for this
    def predict_collision(self, env):
        for first_car, second_car in itertools.combinations(env.vehicles, 2):
            if self.collision(first_car.future_waypoints, second_car.future_waypoints):
                print("collision between", first_car.id, "and", second_car.id)