#! /usr/bin/env python3

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler
from frenet import *
from geometry_utils import *
from lane_info import *
from collision_predictor.msg import *

class Subscriber:
    def __init__(self):
        # position subcribers
        self.sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.callback1)
        self.sub2 = rospy.Subscriber('/tb3_2/odom', Odometry, self.callback2)
        self.sub3 = rospy.Subscriber('/tb3_3/odom', Odometry, self.callback3)
        self.sub4 = rospy.Subscriber('/tb3_4/odom', Odometry, self.callback4)
        self.sub5 = rospy.Subscriber('/tb3_5/odom', Odometry, self.callback5)

    # collision check
    def lineIntersection(self, future_waypoints_1, future_waypoints_2):
        intersect = Point()
        p0_x = future_waypoints_1[0].x
        p0_y = future_waypoints_1[0].y
        p1_x = future_waypoints_1[-1].x
        p1_y = future_waypoints_1[-1].y
        p2_x = future_waypoints_2[0].x
        p2_y = future_waypoints_2[0].y
        p3_x = future_waypoints_2[-1].x
        p3_y = future_waypoints_1[-1].y

        s1_x = p1_x - p0_x
        s1_y = p1_y - p0_y
        s2_x = p3_x - p2_x
        s2_y = p3_y - p2_y

        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y))/(-s2_x * s1_y + s1_x * s2_y)
        t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x))/(-s2_x * s1_y + s1_x * s2_y)

        if s >= 0 and s <= 1 and t >=0 and t <= 1:
            # collision detected
            intersect.x = p0_x + (t * s1_x)
            intersect.y = p0_y + (t * s1_y)
            intersect.z = 0.0
            print("!!COLLISION AHEAD!!")
            return True
        
        return False  # no collision

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
        # adding the last point
        pose_arr.append(PoseStamped(Header, Pose(Point2D(lane_route[-1][0], lane_route[-1][1]), quat)))
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
            p3 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            p3 = Point2D(path[ind_closest + 2].x, path[ind_closest + 2].y)
            prev_idx = ind_closest
        
        return p1, p2, p3, prev_idx

    # adding on theta to get the yaw of the vehicle
    def get_frenet_with_theta(self, x, y, path, s_map):
        if path == None:
            print("Empty map. Cannot return Frenet coordinates")
            return 0.0, 0.0, 0.0

        ind_closest = closest_point_ind(path, x, y)

        # Determine the indices of the 2 closest points
        if ind_closest < len(path):
            p1, p2, _, prev_idx = self.closest_points(ind_closest, path, x, y)
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

    # future waypoints
    def PredictTrajectoryVehicles(self, init_x, init_y, path, s_map, v, d):   
        s, d_curr, _ = self.get_frenet_with_theta(init_x, init_y, path, s_map)
        d = (d_curr + d) / 2                    # average of all the deviations from center
        future_waypoints = []
        for t in range(self.np_m):
            if t < self.interp_back_path:
                d_val = d - ((t*d) / self.interp_back_path)
                new_x, new_y = self.get_xy(s+v*self.dt_m*t, d_val, path, s_map)
            else:
                new_x, new_y = self.get_xy(s+v*self.dt_m*t, 0, path, s_map)
            future_waypoints.append(Point(new_x, new_y, 0,0))
        return future_waypoints, d

    # getting the future trajectory
    def get_future_trajectory(self, car):    
        v = np.mean(car.past_vel)
        d = np.mean(car.past_d)
        lane_line_list, lane_s_map = self.get_lane_and_s_map(car.car_route)
        future_waypoints, d = self.PredictTrajectoryVehicles(car.pose.pose.pose.position.x, car.pose.pose.pose.position.y, lane_line_list, lane_s_map, v, d)

        return future_waypoints, d

    def callback1(self, msg):
        self.car_1_pose = msg.pose.pose

    def callback2(self, msg):
        self.car_2_pose = msg.pose.pose

    def callback(self, msg):
        self.car_pose = msg

    def update(self, car):
        path, _  = self.get_lane_and_s_map(car.car_route)
        x, y = car.pose.pose.pose.position.x, car.pose.pose.pose.position.y
        ind_closest = closest_point_ind(path, x, y)
        # still on the lane
        if ind_closest < len(path)-2:
            p1, p2, p3, _ = self.closest_points(ind_closest, path, x, y)

            yaw = math.atan((p2.y - p1.y)/(p2.x - p1.x))
            next_yaw = math.atan((p3.y - p2.y)/ (p3.x - p2.x))
            v = car.twist.linear.x
            linear = Vector3(v*math.cos(yaw), v*math.sin(yaw), 0)
            angular = Vector3(0, 0, 3*(next_yaw - yaw)/self.dt_m)
            move = Twist(linear, angular)
            car.stop = False
        # stop after reaching the end of lane
        else:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)
            move = Twist(linear, angular)
            car.stop = True

        car.pose = self.callback()  # think about this tomorrow, how to callback smartly
        car.twist = car.pose.twist
        car.past_vel.pop(0)
        car.past_vel.append(v)
        car.past_d.pop(0)
        car.past_d.append(car.d)   
        return move

    def add(self, car):
        env.register = True
        env.vehicles += 1
        env.vehicle_states.append(car)

    def removal(self, car):
        env.deregister = True
        env.vehicles -= 1
        env.vehicle_states.remove(car)

    def main(self):
        while not rospy.is_shutdown():
            if not car_1.future_waypoints and not car_2.future_waypoints:
                print("there's no future trajectory, vehicle has just started interacting with the environment")
                print("time to register the vehicle into the environment")
                env.add(car_1)
                env.add(car_2)
            else:
                if not self.lineIntersection(car_1.future_waypoints, car_2.future_waypoints):
                    # start checking for intersection
                    car_1.future_waypoints, car_1.d = self.get_future_trajectory(car_1)
                    car_2.future_waypoints, car_2.d = self.get_future_trajectory(car_2)
                    move1 = self.update(car_1)
                    move2 = self.update(car_2)
                    pub1.publish(move1)
                    pub2.publish(move2)
                    if car_1.stop:
                        self.removal(car_1)
                    if car_2.stop:
                        self.removal(car_2)
                # add more functionalities
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        # registering the vehicles
        pos_car_1 = Point(-4.0, 0.0, 0.0)
        yaw_car_1 = 0
        lin_vel_1 = 0.4
        ang_vel_1 = 0.0
        car_1_pose = Pose(pos_car_1, quaternion_from_euler(0, 0, yaw_car_1))
        car_1_twist = Twist(lin_vel_1, ang_vel_1)
        s_car_1 = 0 
        d_car_1 = 0.0
        past_vel_1 = [lin_vel_1]*10
        past_d_1 = [d_car_1]*10
        car_1_odom = Odometry(Header, car_1_pose, car_1_twist) 
        stop_1 = False  
        future_waypoints_1 = [] 

        pos_car_2 = Point(8.0, -2.0, 0.0)
        yaw_car_2 = 2.36
        lin_vel_2 = 0.1
        ang_vel_2 = 0.0
        car_2_pose = Pose(pos_car_2, quaternion_from_euler(0, 0, yaw_car_2))
        car_2_twist = Twist(lin_vel_2, ang_vel_2)
        s_car_2 = 0 
        d_car_2 = 0.0
        past_vel_2 = [lin_vel_2]*10
        past_d_2 = [d_car_2]*10
        car_2_odom = Odometry(Header, car_2_pose, car_2_twist) 
        stop_2 = False  
        future_waypoints_2 = []

        # initialize the vehicles
        car_1 = VehicleState("car1", car_1_odom, car_1_twist, past_vel_1, d_car_1, past_d_1, stop_1, future_waypoints_1, car_1_route)
        car_2 = VehicleState("car2", car_2_odom, car_2_twist, past_vel_2, d_car_2, past_d_2, stop_2, future_waypoints_2, car_2_route)
        car_3 = VehicleState()
        car_4 = VehicleState()
        car_5 = VehicleState()

        # environment setup
        no_of_vehicles = 0
        vehicle_list = []
        at_junction = False
        register = False
        deregister = False 
        interaction = False
        env = Environment(no_of_vehicles, vehicle_list, at_junction, register, deregister, interaction)

        rospy.init_node('predictor', anonymous=True)
        pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
        pub4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=10)
        pub5 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=10)
        sub = Subscriber()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass