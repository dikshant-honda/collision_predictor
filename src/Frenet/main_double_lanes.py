#! /usr/bin/env python3

import rospy
import math
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import itertools
import message_filters
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Vector3, PoseWithCovariance, Quaternion
from collision_predictor.msg import Environment, VehicleState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from controllers.controller import Controller
from helper.frenet import *
from env_info.lane_info import LaneInfo
from Frenet.plotter import plotter

class Subscriber:
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
        
        # subscribers
        self.car_1_sub = message_filters.Subscriber('/car_1/odom', Odometry)
        self.car_2_sub = message_filters.Subscriber('/car_2/odom', Odometry)
        self.car_3_sub = message_filters.Subscriber('/car_3/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.car_1_sub, self.car_2_sub, self.car_3_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        # controller for handling dynamics of the vehicles
        self.control = Controller()
        
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
        # car.d = d
        return future_waypoints

    def publishers(self, car, move):
        if car.id == "car_1":
            pub1.publish(move)
        if car.id == "car_2":
            pub2.publish(move)
        if car.id == "car_3":
            pub3.publish(move)

    def move(self, car):
        # move one step using PI yaw controller
        car, twist = self.control.step(car)
        # publish the twist message
        self.publishers(car, twist)

    # time synchronized callback
    def callback(self, veh_1, veh_2, veh_3):
        # car 1 updates
        car_1.pose = veh_1

        # car 2 updates
        car_2.pose = veh_2

        # car 3 updates
        car_3.pose = veh_3

    def stop(self, car):
        print("!!! Stop:", car.id, "!!!")
        linear = Vector3(0, 0, 0)
        angular = Vector3(0, 0, 0)
        move = Twist(linear, angular)
        car.stop = True
        self.publishers(car, move)

    # collision check by vicinity or point-wise check
    def collision(self, points1, points2):
        for i in range(len(points1)):
            for j in range(len(points2)):
                if distance(points1[i].x, points1[i].y, points2[j].x, points2[j].y) < self.tol:
                    return True
        return False

    def point_to_arr_write(self, car, points_arr):
        file_name = "traj_"+car+"_double.txt"
        file = open(os.path.join(save_path, file_name), "w")
        for i in range(len(points_arr)):
            file.write(str(points_arr[i].x))
            file.write("\t")
            file.write(str(points_arr[i].y))
            file.write("\n")
        # print("trajectory for", car, "added to file:", file_name)
        file.close()

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

    def add_to_intersection(self, car, junction_type):
        if car not in self.car_at_junction[junction_type]:
            self.car_at_junction[junction_type].append(car)

    def remove_from_intersection(self, car, junction_type):
        if car in self.car_at_junction[junction_type]:
            self.car_at_junction[junction_type].remove(car)

    def at_intersection(self, car):
        car_pos = car.pose.pose.pose.position
        T_intersection_origin = Point(-5.5, 0, 0)
        X_intersection_origin = Point(0, 0, 0)
        Y_intersection_origin = Point(6, 0, 0)

        d_T = distance(car_pos.x, car_pos.y, T_intersection_origin.x, T_intersection_origin.y)
        d_X = distance(car_pos.x, car_pos.y, X_intersection_origin.x, X_intersection_origin.y)
        d_Y = distance(car_pos.x, car_pos.y, Y_intersection_origin.x, Y_intersection_origin.y)
        
        if d_T <= self.intersection_vision and car.at_lanes:
            # print("Vehicle at the T-intersection")
            self.add_to_intersection(car, "T")
            car.at_junction = True
            car.at_lanes = False

        if d_X <= self.intersection_vision and car.at_lanes:
            # print("Vehicle at the X-intersection")
            self.add_to_intersection(car, "X")
            car.at_junction = True
            car.at_lanes = False
        
        if d_Y <= self.intersection_vision and car.at_lanes:
            # print("Vehicle at the Y-intersection")
            self.add_to_intersection(car, "Y")
            car.at_junction = True
            car.at_lanes = False

        if  d_T >= self.intersection_vision:
            self.remove_from_intersection(car, "T")
            car.at_lanes = True
            car.at_junction = False
            
        if  d_X >= self.intersection_vision:
            self.remove_from_intersection(car, "X")
            car.at_lanes = True
            car.at_junction = False

        if  d_Y >= self.intersection_vision:
            self.remove_from_intersection(car, "Y")
            car.at_lanes = True
            car.at_junction = False

    def plot_current_position(self):
        car_1_pos = car_1.pose.pose.pose.position
        car_2_pos = car_2.pose.pose.pose.position
        car_3_pos = car_3.pose.pose.pose.position

        plt.plot(car_1_pos.x, car_1_pos.y, 'r*')
        plt.plot(car_2_pos.x, car_2_pos.y, 'c*')
        plt.plot(car_3_pos.x, car_3_pos.y, 'g*')

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

    def get_route(self, pos, original_lane, next_lane):
        lane = lanes.stack_lanes(original_lane, next_lane)
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
    
    def arriving_near_intersection(self, car, car_pos, intersection_center):
        arriving = False
        x, y = car_pos.x, car_pos.y
        dist = distance(x, y, intersection_center[0], intersection_center[1])
        while 2.5 < dist < 5 and not car.at_junction:
            if 4.9 < dist < 5:
                print("coming towards the intersection, sample multiple trajectories")
            arriving = True
            return arriving
        if 2.0 < dist < 2.5 and not car.at_junction:
            print("*******************************************************")
            print(car.id, "reached intersection, sample one of the trajectory")
            print("*******************************************************")
            car.at_junction = True
            next_routes = lanes.get_turning_routes(car.location)
            idx = np.random.randint(0,3)
            if idx == 0:
                print(car.id, ": turn left")
            elif idx == 1:
                print(car.id, ": go straight")
            else:
                print(car.id, ": turn right")
            # idx = 2     # for testing
            chosen_route = next_routes[idx]
            merging_route = lanes.get_linking_route(chosen_route)
            car.location = lanes.stack_lanes(car.location, chosen_route)
            car.location = lanes.stack_lanes(car.location, merging_route)
        return arriving
    
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

        # register vehicles to the environment
        self.add(car_1)
        self.add(car_2)
        self.add(car_3)

        while not rospy.is_shutdown():
            start = time.time()

            # print current position of the vehicle
            self.plot_current_position()

            # update the environment info and move
            for car in env.vehicle_states:
                if not car.reached_end:
                    self.update(car)
                    self.move(car)
                else:
                    self.stop(car)

            self.predict_collision()

            end = time.time()
            time_taken += end-start

            # plotting tools
            plt.xlim(-13, 13)
            plt.ylim(-13, 13)
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

        # registering the vehicles
        # car 1 information
        pos_car_1 = Point(-0.9, -10.0, 0.0)
        yaw_car_1 = 1.57
        v_1 = 0.7
        lin_vel_1 = Vector3(v_1, 0.0, 0.0)
        ang_vel_1 = Vector3(0.0, 0.0, 0.0)
        q_1 = quaternion_from_euler(0, 0, yaw_car_1)
        car_1_pose = Pose(pos_car_1, Quaternion(q_1[0], q_1[1], q_1[2], q_1[3]))
        car_1_twist = Twist(lin_vel_1, ang_vel_1)
        covariance_1 = [[0 for _ in range(6)] for _ in range(6)]
        car_1_pose_with_covariance = PoseWithCovariance(car_1_pose, covariance_1)
        car_1_odom = Odometry(Header, "base_footprint", car_1_pose_with_covariance, car_1_twist) 
        stop_1 = False  
        future_waypoints_1 = []
        reached_end_1 = False
        at_junction_1 = False
        location_1 = lanes.lane_5
        car_1_route_ = []
        car_1_yaw_ = []

        # car 2 information
        pos_car_2 = Point(9.0, -0.9, 0.0)
        yaw_car_2 = 3.14
        v_2 = 0.6
        lin_vel_2 = Vector3(v_2, 0.0, 0.0)
        ang_vel_2 = Vector3(0.0, 0.0, 0.0)
        q_2 = quaternion_from_euler(0, 0, yaw_car_2)
        car_2_pose = Pose(pos_car_2, Quaternion(q_2[0], q_2[1], q_2[2], q_2[3]))
        car_2_twist = Twist(lin_vel_2, ang_vel_2)
        covariance_2 = [[0 for _ in range(6)] for _ in range(6)]
        car_2_pose_with_covariance = PoseWithCovariance(car_2_pose, covariance_2)
        car_2_odom = Odometry(Header, "base_footprint", car_2_pose_with_covariance, car_2_twist) 
        stop_2 = False  
        future_waypoints_2 = []
        reached_end_2 = False
        at_junction_2 = False
        location_2 = lanes.lane_1
        car_2_route_ = []
        car_2_yaw_ = []

        # car 3 information
        pos_car_3 = Point(-10.0, 0.9, 0.0)
        yaw_car_3 = 0.0
        v_3 = 0.7
        lin_vel_3 = Vector3(v_3, 0.0, 0.0)
        ang_vel_3 = Vector3(0.05, 0.0, 0.0)
        q_3 = quaternion_from_euler(0, 0, yaw_car_3)
        car_3_pose = Pose(pos_car_3, Quaternion(q_3[0], q_3[1], q_3[2], q_3[3]))
        car_3_twist = Twist(lin_vel_3, ang_vel_3)
        covariance_3 = [[0 for _ in range(6)] for _ in range(6)]
        car_3_pose_with_covariance = PoseWithCovariance(car_3_pose, covariance_3)
        car_3_odom = Odometry(Header, "base_footprint", car_3_pose_with_covariance, car_3_twist) 
        stop_3 = False  
        future_waypoints_3 = []
        reached_end_3 = False
        at_junction_3 = False
        location_3 = lanes.lane_4
        car_3_route_ = []
        car_3_yaw_ = []

        # initialize the vehicles
        car_1 = VehicleState("car_1", car_1_odom, v_1, stop_1, future_waypoints_1, car_1_route_, car_1_yaw_, reached_end_1, at_junction_1, location_1)
        car_2 = VehicleState("car_2", car_2_odom, v_2, stop_2, future_waypoints_2, car_2_route_, car_2_yaw_, reached_end_2, at_junction_2, location_2)
        car_3 = VehicleState("car_3", car_3_odom, v_3, stop_3, future_waypoints_3, car_3_route_, car_3_yaw_, reached_end_3, at_junction_3, location_3)

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

        rospy.init_node('predictor', anonymous=True)
        pub1 = rospy.Publisher('/car_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/car_2/cmd_vel', Twist, queue_size=10)
        pub3 = rospy.Publisher('/car_3/cmd_vel', Twist, queue_size=10)

        sub = Subscriber()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass