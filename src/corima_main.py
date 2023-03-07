#! /usr/bin/env python3

import rospy
import time
import numpy as np
import message_filters
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Vector3, PoseWithCovariance, Quaternion
from collision_predictor.msg import Environment, VehicleState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from frenet import *
from lane_info_2 import LaneInfo
from controller import Controller
from corima_wrapper.predict_collisions import predict_collisions
from corima_wrapper.model import DataPoint
from corima_wrapper.model import Position, Velocity

class Subscriber:
    def __init__(self):
        # subscribers
        self.car_1_sub = message_filters.Subscriber('/car_1/odom', Odometry)
        self.car_2_sub = message_filters.Subscriber('/car_2/odom', Odometry)
        self.car_3_sub = message_filters.Subscriber('/car_3/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.car_1_sub, self.car_2_sub, self.car_3_sub], 10, 0.1)
        self.ts.registerCallback(self.callback)

        # controller for handling dynamics of the vehicles
        self.control = Controller()

        self.main()

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
    
    def publishers(self, car, move):
        if car.id == "car_1":
            pub1.publish(move)
        if car.id == "car_2":
            pub2.publish(move)
        if car.id == "car_3":
            pub3.publish(move)

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
            print(car.id, "reached at the intersection, sample one of the trajectory")
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
            idx = 2     # for testing
            chosen_route = next_routes[idx]
            merging_route = lanes.get_linking_route(chosen_route)
            car.location = lanes.stack_lanes(car.location, chosen_route)
            car.location = lanes.stack_lanes(car.location, merging_route)
        return arriving
    
    def update(self, car, predictions):
        if self.arriving_near_intersection(car, car.pose.pose.pose.position, [0, 0]) and not car.at_junction:
            # get route from the current position of the vehicle
            possible_lanes = lanes.get_turning_routes(car.location)
            for lane in possible_lanes:
                car_route_, yaw_route_ = self.get_route(car.pose.pose.pose.position, car.location, lane)
                car.car_route = car_route_
                car.car_yaw = yaw_route_
            predictions.append(self.register_to_corima(car))    # indent to check in all directions

        else:
            car_route_, yaw_route_ = self.get_route(car.pose.pose.pose.position, car.location, [])
            car.car_route = car_route_
            car.car_yaw = yaw_route_

            if len(car_route_) <= 2:
                print("reached the end point")
                self.stop(car)
                self.EOL(car)
                car.reached_end = True
            else:
                predictions.append(self.register_to_corima(car))

    def register_to_corima(self, car):
        type_ = "car"
        yaw = car.car_yaw[0]
        v = np.sqrt(car.twist.linear.x**2 + car.twist.linear.y**2)
        velocity = Velocity(v*np.cos(yaw), v*np.sin(yaw))
        position = Position(car.pose.pose.pose.position.x, car.pose.pose.pose.position.y)
        id = car.id
        route = car.car_route
        point = DataPoint(id, position, velocity, route, type_)
        return point
    
    def move(self, car):
        # move one step using PI yaw controller
        car, twist = self.control.step(car)
        # publish the twist message
        self.publishers(car, twist)

    def main(self):
        time_taken = 0

        # register vehicles to the environment
        self.add(car_1)
        self.add(car_2)
        self.add(car_3)

        while not rospy.is_shutdown():
            start = time.time()

            predictions = []

            # update the environment info
            for car in env.vehicle_states:
                if not car.reached_end:
                    self.update(car, predictions)
                else:
                    self.stop(car)
                    # self.EOL(car)
            
            # replace in real world
            if len(predictions) == 1:
                break

            # predict collision probability
            # result = [[car.id, [prob1, prob2, ...]]]
            result = predict_collisions(predictions)
            
            # replace this by alert alarms
            # for time being, stop car_1 and give priority to 3 between 2 and 3
            if max(result[0][1]) > 0.01:
                self.stop(car_1)
                if result[1][1][1] > 0.01:
                    self.stop(car_2)
                else:
                    self.move(car_2)
            else:
                self.move(car_1)
                self.move(car_2)

            self.move(car_3)
            
            end = time.time()
            time_taken += end-start

            print("Loop execution time", end-start)
            print("time elapsed:", time_taken)
            print("------------------------------------------")
            if env.vehicles == 0:
                print("Execution Done")
                break   

if __name__ == '__main__':
    try:
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
        location_3 = lanes.lane_4
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

        # nodes for controlling the motion of the vehicles
        rospy.init_node('control', anonymous=True)
        pub1 = rospy.Publisher('/car_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/car_2/cmd_vel', Twist, queue_size=10)
        pub3 = rospy.Publisher('/car_3/cmd_vel', Twist, queue_size=10)

        sub = Subscriber() 

        rospy.spin()

    except rospy.ROSInterruptException:
        pass