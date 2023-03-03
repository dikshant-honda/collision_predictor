#! /usr/bin/env python3

import rospy
import math
import time
import os
import numpy as np
import matplotlib.pyplot as plt
import message_filters
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Vector3, PoseWithCovariance, Quaternion
from collision_predictor.msg import Environment, VehicleState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from frenet import *
from geometry_utils import *
from lane_info_2 import *
from pid_planner import PI
from plotter import plotter
from corima_wrapper.predict_collisions import predict_collisions
from corima_wrapper.model import DataPoint
from corima_wrapper.model import Position, Velocity

class Subscriber:
    def __init__(self):
        # variables
        self.interp_back_path = 1000                    # interpolate back to path after this # of steps
        self.plan_t_m = 5                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 0.1                                  # tolerance value for proximity check
        self.vision_radius = 3                          # check only nearby cars
      
        # subscribers
        self.car_1_sub = message_filters.Subscriber('/car_1/odom', Odometry)
        self.car_2_sub = message_filters.Subscriber('/car_2/odom', Odometry)
        self.car_3_sub = message_filters.Subscriber('/car_3/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.car_1_sub, self.car_2_sub, self.car_3_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        self.main()

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
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest
        
        return p1, p2, prev_idx

    # adding on theta to get the yaw of the vehicle
    def get_frenet_with_theta(self, x, y, path, s_map):
        if path == None:
            print("Empty map. Cannot return Frenet coordinates")
            return 0.0, 0.0, 0.0

        ind_closest = closest_point_ind(path, x, y)

        # Determine the indices of the 2 closest points
        if ind_closest < len(path):
            p1, p2, prev_idx = self.closest_points(ind_closest, path, x, y)
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

    def publishers(self, car, move):
        if car.id == "car_1":
            pub1.publish(move)
        if car.id == "car_2":
            pub2.publish(move)
        if car.id == "car_3":
            pub3.publish(move)

    def correct_angle(self, angle):
        if angle > np.pi:
            angle = angle - 2*np.pi
        elif angle < -np.pi:
            angle = angle + 2*np.pi
        
        return angle

    def move(self, car):
        path, _  = self.get_lane_and_s_map(car.car_route)
        x_pos, y_pos = car.pose.pose.pose.position.x, car.pose.pose.pose.position.y
        ind_closest = closest_point_ind(path, x_pos, y_pos)        
        yaw_path = car.car_yaw
        # still on the lane
        if ind_closest < len(path)-1:
            x, y, z, w = car.pose.pose.pose.orientation.x, car.pose.pose.pose.orientation.y, car.pose.pose.pose.orientation.z, car.pose.pose.pose.orientation.w
            _, _, init_yaw = euler_from_quaternion([x, y, z, w])

            # PI controller for yaw correction
            pi = PI(P=14.14, I = 1000)
            yaw_desired = yaw_path[ind_closest]
            feedback = self.correct_angle(init_yaw)
            ang_error = yaw_desired - feedback
            ang_error = self.correct_angle(ang_error)
            pi.update(-ang_error)
            omega = pi.output
            
            v = np.mean(car.past_vel) 

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
            car.stop = True
            # self.EOL(car)

        # publish the move message
        self.publishers(car, move)

    # time synchronized callback
    def callback(self, veh_1, veh_2, veh_3):
        # car 1 updates
        car_1.pose = veh_1
        car_1.twist = veh_1.twist.twist
        car_1.past_vel.pop(0)
        car_1.past_vel.append(v_1)
        car_1.past_d.pop(0)
        car_1.past_d.append(car_1.d)

        # car 2 updates
        car_2.pose = veh_2
        car_2.twist = veh_2.twist.twist
        car_2.past_vel.pop(0)
        car_2.past_vel.append(v_2)
        car_2.past_d.pop(0)
        car_2.past_d.append(car_2.d)

        # car 3 updates
        car_3.pose = veh_3
        car_3.twist = veh_3.twist.twist
        car_3.past_vel.pop(0)
        car_3.past_vel.append(v_3)
        car_3.past_d.pop(0)
        car_3.past_d.append(car_3.d)

    def stop(self, car):
        print("!!! Stop:", car.id, "!!!")
        linear = Vector3(0, 0, 0)
        angular = Vector3(0, 0, 0)
        move = Twist(linear, angular)
        car.stop = True
        self.publishers(car, move)

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

    def plot_current_position(self):
        car_1_pos = car_1.pose.pose.pose.position
        car_2_pos = car_2.pose.pose.pose.position
        car_3_pos = car_3.pose.pose.pose.position

        plt.plot(car_1_pos.x, car_1_pos.y, 'r*')
        plt.plot(car_2_pos.x, car_2_pos.y, 'c*')
        # plt.plot(car_3_pos.x, car_3_pos.y, 'g*')

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

    def stack_lanes(self, prev_lane, next_lane):
        if len(next_lane) == 0:
            return prev_lane
        prev_arr_x, prev_arr_y = point_to_arr(prev_lane)
        next_arr_x, next_arr_y = point_to_arr(next_lane)
        lane_x = np.hstack((prev_arr_x, next_arr_x))
        lane_y = np.hstack((prev_arr_y, next_arr_y))
        lane = [arr_to_point(lane_x, lane_y), np.hstack((prev_lane[1], next_lane[1]))]
        return lane
    
    def get_route(self, pos, original_lane, next_lane):
        lane = self.stack_lanes(original_lane, next_lane)
        idx = self.closest_pt_idx(pos.x, pos.y, lane)
        car_route_ = []
        yaw_route_ = []
        horizon = 0
        while idx < len(lane[0]) and horizon < 300:
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
        if 2.4 < dist < 2.5 and not car.at_junction:
            print("*******************************************************")
            print("reached intersection, sample one of the trajectory")
            print("*******************************************************")
            car.at_junction = True
            next_routes = get_turning_routes(car.location)
            idx = np.random.randint(0,3)
            if idx == 0:
                print(car.id, ": turn left")
            elif idx == 1:
                print(car.id, ": go straight")
            else:
                print(car.id, ": turn right")
            idx = 0
            chosen_route = next_routes[idx]
            merging_route = get_linking_route(chosen_route)
            car.location = self.stack_lanes(car.location, chosen_route)
            car.location = self.stack_lanes(car.location, merging_route)
        return arriving
    
    def update(self, car):
        if self.arriving_near_intersection(car, car.pose.pose.pose.position, [0, 0]) and not car.at_junction:
            # get route from the current position of the vehicle
            possible_lanes = get_turning_routes(car.location)
            for lane in possible_lanes:
                car_route_, yaw_route_ = self.get_route(car.pose.pose.pose.position, car.location, lane)
                car.car_route = car_route_
                car.car_yaw = yaw_route_

                # car.future_waypoints = self.get_future_trajectory(car)

                # self.plot_future_trajectory(car)
        else:
            car_route_, yaw_route_ = self.get_route(car.pose.pose.pose.position, car.location, [])
            car.car_route = car_route_
            car.car_yaw = yaw_route_

            # if len(car_route_) <= 2:
            #     print("reached the end point")
            #     self.stop(car)
            #     self.EOL(car)
            #     car.reached_end = True
            # else:
            #     car.future_waypoints = self.get_future_trajectory(car)

            # self.plot_future_trajectory(car)

    def corima_collision_predictor(self):
        poses = []
        type_ = "car"
        velocity_1 = Velocity(0, 0.7)
        position_1 = Position(car_1.pose.pose.pose.position.x, car_1.pose.pose.pose.position.y)
        id_1 = "14"
        velocity_2 = Velocity(0.6, 0)
        position_2 = Position(car_2.pose.pose.pose.position.x, car_2.pose.pose.pose.position.y)
        id_2 = "7"
        pt_1 = DataPoint(id_1, position_1, velocity_1, type_)
        pt_2 = DataPoint(id_2, position_2, velocity_2, type_)
        poses.append(pt_1)
        poses.append(pt_2)
        result = predict_collisions(poses)
        return result

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

            result = self.corima_collision_predictor()
            # print(result)
                
            # update the environment info and move
            if not car_1.reached_end:
                self.update(car_1)
                self.move(car_1)
            # print(result)

            if not car_2.reached_end:
                self.update(car_2)
                self.move(car_2)

            # if not car_3.reached_end:
            #     self.update(car_3)
            #     self.move(car_3)

            # if self.collision(car_1.future_waypoints, car_2.future_waypoints):
            #     print("possibility of collision")
            #     self.stop(car_2)
            
            # if self.collision(car_1.future_waypoints, car_3.future_waypoints):
            #     print("possibility of collision")
            #     self.stop(car_3)

            # if self.collision(car_2.future_waypoints, car_3.future_waypoints):
            #     print("possibility of collision")
            #     self.stop(car_2)
            
            end = time.time()
            time_taken += end-start

            # plotting tools
            # plt.xlim(-13, 13)
            # plt.ylim(-13, 13)
            # plt.xlabel("X")
            # plt.ylabel("Y")
            # plt.title("Trajectories of the moving vehicles")
            # plt.pause(0.000000001)

            print("Loop execution time", end-start)
            print("time elapsed:", time_taken)
            print("------------------------------------------")
            if env.vehicles == 0:
                print("Execution Done")
                break   

if __name__ == '__main__':
    try:
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
        s_car_1 = 0 
        d_car_1 = 0.0
        past_vel_1 = [v_1]*10
        past_d_1 = [d_car_1]*10
        covariance_1 = [[0 for _ in range(6)] for _ in range(6)]
        car_1_pose_with_covariance = PoseWithCovariance(car_1_pose, covariance_1)
        car_1_odom = Odometry(Header, "base_footprint", car_1_pose_with_covariance, car_1_twist) 
        stop_1 = False  
        future_waypoints_1 = []
        reached_end_1 = False
        at_junction_1 = False
        location_1 = lane_5
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
        s_car_2 = 0 
        d_car_2 = 0.0
        past_vel_2 = [v_2]*10
        past_d_2 = [d_car_2]*10
        covariance_2 = [[0 for _ in range(6)] for _ in range(6)]
        car_2_pose_with_covariance = PoseWithCovariance(car_2_pose, covariance_2)
        car_2_odom = Odometry(Header, "base_footprint", car_2_pose_with_covariance, car_2_twist) 
        stop_2 = False  
        future_waypoints_2 = []
        reached_end_2 = False
        at_junction_2 = False
        location_2 = lane_1
        car_2_route_ = []
        car_2_yaw_ = []

        # car 3 information
        pos_car_3 = Point(-10.0, 0.9, 0.0)
        yaw_car_3 = 0.0
        v_3 = 0.7
        lin_vel_3 = Vector3(v_3, 0.0, 0.0)
        ang_vel_3 = Vector3(0.0, 0.0, 0.0)
        q_3 = quaternion_from_euler(0, 0, yaw_car_3)
        car_3_pose = Pose(pos_car_3, Quaternion(q_3[0], q_3[1], q_3[2], q_3[3]))
        car_3_twist = Twist(lin_vel_3, ang_vel_3)
        s_car_3 = 0 
        d_car_3 = 0.0
        past_vel_3 = [v_3]*10
        past_d_3 = [d_car_3]*10
        covariance_3 = [[0 for _ in range(6)] for _ in range(6)]
        car_3_pose_with_covariance = PoseWithCovariance(car_3_pose, covariance_3)
        car_3_odom = Odometry(Header, "base_footprint", car_3_pose_with_covariance, car_3_twist) 
        stop_3 = False  
        future_waypoints_3 = []
        reached_end_3 = False
        at_junction_3 = False
        location_3 = lane_4
        car_3_route_ = []
        car_3_yaw_ = []

        # initialize the vehicles
        car_1 = VehicleState("car_1", car_1_odom, car_1_twist, past_vel_1, d_car_1, past_d_1, stop_1, future_waypoints_1, car_1_route_, car_1_yaw_, reached_end_1, at_junction_1, location_1)
        car_2 = VehicleState("car_2", car_2_odom, car_2_twist, past_vel_2, d_car_2, past_d_2, stop_2, future_waypoints_2, car_2_route_, car_2_yaw_, reached_end_2, at_junction_2, location_2)
        car_3 = VehicleState("car_3", car_3_odom, car_3_twist, past_vel_3, d_car_3, past_d_3, stop_3, future_waypoints_3, car_3_route_, car_3_yaw_, reached_end_3, at_junction_3, location_3)

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