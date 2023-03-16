#! /usr/bin/env python3

import rospy
import time
import numpy as np
import message_filters
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose, Vector3, PoseWithCovariance, Quaternion
from collision_predictor.msg import Environment, VehicleState
from tf.transformations import quaternion_from_euler
from helper.frenet import distance
from env_info.lane_info import LaneInfo
from env_info.vehicle_info import VehicleInfo
from controllers.controller import Controller
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
        # car_1.twist = veh_1.twist.twist

        # car 2 updates
        car_2.pose = veh_2
        # car_2.twist = veh_2.twist.twist

        # car 3 updates
        car_3.pose = veh_3
        # car_3.twist = veh_3.twist.twist
    
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
        v = np.sqrt(car.vel)
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
        # get lane info
        lanes = LaneInfo()

        # get  vehicle info
        vehicles = VehicleInfo()

        # initialzing the vehicles
        car_1 = vehicles.car_1
        car_2 = vehicles.car_2
        car_3 = vehicles.car_3
        
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