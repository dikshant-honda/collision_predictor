#! /usr/bin/env python3

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose, Vector3, PoseWithCovariance, Quaternion
from collision_predictor.msg import  VehicleState
from tf.transformations import quaternion_from_euler
from lane_info import LaneInfo

class VehicleInfo:
    def __init__(self) -> None:
        # --------------- defining all the vehicles -------------
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
        self.car_1 = VehicleState("car_1", car_1_odom, car_1_twist, past_vel_1, d_car_1, past_d_1, stop_1, future_waypoints_1, car_1_route_, car_1_yaw_, reached_end_1, at_junction_1, location_1)
        self.car_2 = VehicleState("car_2", car_2_odom, car_2_twist, past_vel_2, d_car_2, past_d_2, stop_2, future_waypoints_2, car_2_route_, car_2_yaw_, reached_end_2, at_junction_2, location_2)
        self.car_3 = VehicleState("car_3", car_3_odom, car_3_twist, past_vel_3, d_car_3, past_d_3, stop_3, future_waypoints_3, car_3_route_, car_3_yaw_, reached_end_3, at_junction_3, location_3)