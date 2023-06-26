#! /usr/bin/env python3

import math
import time
import matplotlib.pyplot as plt
import itertools
from collision_predictor.msg import Environment
from helper.frenet import *
from env_info.lane_info import LaneInfo
from env_info.vehicle_info import VehicleInfo
from Frenet.plotter import plotter

class Predictions:
    def __init__(self):
        # variables
        self.interp_back_path = 1000                    # interpolate back to path after this # of steps
        self.plan_t_m = 5                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 1.4                                  # tolerance value for proximity check
        self.vision_radius = 3                          # check only nearby cars
        self.intersection_vision = 4                    # check cars arriving near intersection
        self.car_at_junction = {"X":[], "Y":[], "T":[]} # dictionary for storing the ids at the junction

        self.main()

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
        env.register = True
        env.vehicles += 1
        env.vehicle_states.append(car)

    def removal(self, car):
        if car in env.vehicle_states:
            env.deregister = True
            env.vehicles -= 1
            env.vehicle_states.remove(car)

    def EOL(self, car):                         # vehicle has reached the goal point
        self.removal(car)

    def plot_future_trajectory(self, car):
        self.point_to_arr_write(car.id, car.future_waypoints)
        x, y = plotter(car.id)

        # plot trajectories
        plt.plot(x, y, '-')

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

    def update(self, car):
        if self.arriving_near_intersection(car, car.pose.pose.pose.position, [0, 0]) and not car.at_junction:
            # get route from the current position of the vehicle
            possible_lanes = lanes.get_turning_routes(car.location)
            for lane in possible_lanes:
                car_route_, yaw_route_ = self.get_route(car.pose.pose.pose.position, car.location, lane)
                car.car_route = car_route_
                car.car_yaw = yaw_route_

                car.future_waypoints = self.get_future_trajectory(car)

                self.plot_future_trajectory(car)
        else:
            car_route_, yaw_route_ = self.get_route(car.pose.pose.pose.position, car.location, [])
            car.car_route = car_route_
            car.car_yaw = yaw_route_

            if len(car_route_) <= 2:
                print(car.id, "reached the end point")
                self.stop(car)
                self.EOL(car)
                car.reached_end = True
            else:
                car.future_waypoints = self.get_future_trajectory(car)

            self.plot_future_trajectory(car)

    def predict_collision(self):
        for first_car, second_car in itertools.combinations(env.vehicle_states, 2):
            if self.collision(first_car.future_waypoints, second_car.future_waypoints):
                print("collision between", first_car.id, "and", second_car.id)
                self.stop(second_car)

    def main(self):
        time_taken = 0

        while True:
            start = time.time()

            # print current position of the vehicle
            self.plot_current_position()

            # update the environment info and move
            for car in env.vehicle_states:
                if not car.reached_end:
                    self.update(car)

            self.predict_collision()

            end = time.time()
            time_taken += end-start

            # plotting tools
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("Trajectories of the moving vehicles")
            plt.pause(0.000000001)

            print("Loop execution time", end-start)
            print("time elapsed:", time_taken)
            print("------------------------------------------")
            if env.vehicles == 0:
                print("Execution Done")
                break   

if __name__ == '__main__':
    try:
        # get lane info
        lanes = LaneInfo()

        # get  vehicle info
        vehicles = VehicleInfo()

        # environment setup
        no_of_vehicles = 0
        vehicle_states = []
        # at_intersection = False
        register = False
        deregister = False 
        interaction = False
        env = Environment(no_of_vehicles, vehicle_states, register, deregister, interaction)

        # directory for plotting the future trajectories of the vehicles
        save_path = "/home/dikshant/catkin_ws/src/collision_predictor/src"

        predict = Predictions()


    except:
        pass