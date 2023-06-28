#! /usr/bin/env python3

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose, Vector3, PoseWithCovariance, Quaternion
from collision_predictor.msg import  VehicleState
from tf.transformations import quaternion_from_euler
from env_info.lane_info import *

class VehicleInfo:
    def __init__(self) -> None:
        # --------------- defining all the vehicles -------------
        # registering the vehicles
        self.car_1 = self.get_vehicle_state("car_1", [-0.9, -10.0], 1.57, 0.7, 0.0, lane_5)
        self.car_2 = self.get_vehicle_state("car_2", [9.0, -0.9], 3.14, 0.6, 0.0, lane_1)
        self.car_3 = self.get_vehicle_state("car_3", [-10.0, 0.9], 0.0, 0.7, 0.0, lane_4)

    def get_vehicle_state(self, id, pos, yaw, lin_vel, ang_vel, lane):
        # self.id = id
        pos_car = Point(pos[0], pos[1], 0.0)
        yaw_car = yaw
        lin_vel_car = Vector3(lin_vel, 0, 0)
        ang_vel_car = Vector3(0, 0, ang_vel)
        quat = quaternion_from_euler(0, 0, yaw_car)
        pose_car = Pose(pos_car, Quaternion(quat[0], quat[1], quat[2], quat[3]))
        twist_car = Twist(lin_vel_car, ang_vel_car)
        # s_car = 0 
        covariance_car = [[0 for _ in range(6)] for _ in range(6)]
        pose_with_covariance_car = PoseWithCovariance(pose_car, covariance_car)
        odom_car = Odometry(Header, "base_footprint", pose_with_covariance_car, twist_car) 
        stop = False  
        future_waypoints = []
        reached_end = False
        at_junction = False
        location = lane
        route_car_map = []
        yaw_car_map = []

        return VehicleState(id, odom_car, lin_vel, stop, future_waypoints, route_car_map, yaw_car_map, reached_end, at_junction, location)

# serve as a traffic data type for integration
class Traffic:
    def __init__(self, id, pos, vel, theta, type, location, future_waypoints) -> None:
        self.id = id
        self.position = pos
        self.velocity = vel
        self.orientation = theta
        self.type = type
        self.location = location
        self.future_waypoints = future_waypoints 