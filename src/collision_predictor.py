#! /usr/bin/env python3

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from frenet import *
from geometry_utils import *
from lane_info import *

class Vehicle:
    count = 0
    ids = []
    vehicle_dynamics = {}        # [pose, twist, s, d, past, future]
    vision_radius = 1            # register vehicles which are within this radius of the ego vehicle
    def __init__(self, pose, twist, s, d, past_vel, past_d, future_waypoints_x, future_waypoints_y, id):
        self.pose = pose
        self.twist = twist
        self.s = s
        self.d = d
        self.id = id
        self.future_waypoints_x = future_waypoints_x
        self.future_waypoints_y = future_waypoints_y

        # updating the past information
        past_vel.pop(0)
        v = self.twist.linear    # change it later to into x and y directions
        past_vel.append(v)
        past_d.pop(0)
        past_d.append(d)

        self.past_vel = past_vel
        self.past_d = past_d

        # print("****************************************************")
        # print("vehicle type:", id)
        # print("position:", self.pose.position.x, self.pose.position.y)
        # print("velocity:", v)
        # print("offset from center line:", d)

        Vehicle.vehicle_dynamics[self.id] = [self.pose, self.twist, self.s, self.d, self.past_vel, self.past_d, self.future_waypoints_x, self.future_waypoints_y]


    # add the vehicle with unique ids to vehicles array
    def add_vehicle(self, veh):
        if self.id not in Vehicle.ids:
            Vehicle.count += 1
            Vehicle.ids.append(self.id) 

    # register the ego vehicle
    def register_ego(self, ego):
        self.add_vehicle(ego)

    # register the vehicle into dynamics
    def register_vehicle(self, ego, veh):
        if self.ego_vicinity(ego, veh):
            # print("start registering")
            self.add_vehicle(veh)

    # check for the vehicles which are in the vicinity of the ego vehicle
    def ego_vicinity(self, ego, veh):
        ego_pos = ego.pose.position
        veh_pos = veh.pose.position
        if distance(ego_pos.x, ego_pos.y, veh_pos.x, veh_pos.y) < self.vision_radius:
            return True

class Subscriber:
    def __init__(self) -> None:
        # position subcribers
        self.sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.callback)
        self.sub2 = rospy.Subscriber('/tb3_2/odom', Odometry, self.callback)
        self.sub3 = rospy.Subscriber('/tb3_3/odom', Odometry, self.callback)
        self.sub4 = rospy.Subscriber('/tb3_4/odom', Odometry, self.callback)
        self.sub5 = rospy.Subscriber('/tb3_5/odom', Odometry, self.callback)

        # velocity publishers
        self.pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        self.pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
        self.pub4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=10)
        self.pub5 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=10)

        # variables
        self.width = 2                       # lane width
        self.interp_back_path = 15           # interpolate back to path after this # of steps
        self.plan_t_m = 1                    # planning horizon
        self.dt_m = 0.1                      # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)       # number of future waypoints
        self.horizon = 10                    # number of past points info
        self.factor = 10                     # scale down factor for randomness
        
        # lane information data
        #  straight lanes
        self.x1, self.y1 = get_lanes([-6,-1],[-6,-8])     # down right
        self.x2, self.y2 = get_lanes([-6,8], [-6,1])      # down left
        self.x3, self.y3 = get_lanes([-5,0], [-1,0])      # down center
        self.x4, self.y4 = get_lanes([0,5], [0,1])        # center left
        self.x5, self.y5 = get_lanes([0,-1], [0,-5])       # center right
        self.x6, self.y6 = get_lanes([1,0], [5.5,0])      # up center
        self.x7, self.y7 = get_lanes([6.5,-0.5], [10,-4]) # up right
        self.x8, self.y8 = get_lanes([6.5,0.5], [10,4])   # up left

        # splines
        # T intersection
        self.x9, self.y9 = get_spline([-6,-1], [-5,0], np.pi/2,0)
        self.x10, self.y10 = get_spline([-6,-1], [-6,1], np.pi/2, -np.pi/2)
        self.x11,self. y11 = get_spline([-6,1], [-5,0], -np.pi/2, 0)

        # X intersection
        self.x12, self.y12 = get_spline([-1,0], [0,1], 0, np.pi/2)
        self.x13, self.y13 = get_spline([-1,0], [0,-1], 0, -np.pi/2)
        self.x14, self.y14 = get_spline([-1,0], [1,0], 0, 0)
        self.x15, self.y15 = get_spline([0,-1], [1,0], np.pi/2, 0)
        self.x16, self.y16 = get_spline([0,1], [0,-1], -np.pi/2, -np.pi/2)
        self.x17, self.y17 = get_spline([0,1], [1,0], -np.pi/2, 0)

        # Y intersection
        self.x18, self.y18 = get_spline([5.5,0], [6.5, 0.5], 0, np.pi/4)
        self.x19, self.y19 = get_spline([5.5,0], [6.5,-0.5], 0, -np.pi/4)
        self.x20, self.y20 = get_spline([6.5,0.5], [6.5,-0.5], -(np.pi/4+np.pi/2), -np.pi/4)

        # vehicle trajectories
        # car 1 path
        self.x_car_1 = np.hstack((self.x3, self.x14, self.x6, self.x19, self.x7))
        self.y_car_1 = np.hstack((self.y3, self.y14, self.y6, self.y19, self.y7))   

        # car 2 path 
        self.x_car_2 = np.hstack((self.x7[::-1], self.x19[::-1], self.x6[::-1], self.x14[::-1], self.x3[::-1]))
        self.y_car_2 = np.hstack((self.y7[::-1], self.y19[::-1], self.y6[::-1], self.y14[::-1], self.y3[::-1]))

        # car 3 path
        self.x_car_3 = np.hstack((self.x4, self.x17, self.x6, self.x18, self.x8))
        self.y_car_3 = np.hstack((self.y4, self.y17, self.y6, self.y18, self.y8))

        # car 4 path
        self.x_car_4 = np.hstack((self.x1[::-1], self.x9, self.x3, self.x12, self.x4[::-1]))
        self.y_car_4 = np.hstack((self.y1[::-1], self.y9, self.y3, self.y12, self.y4[::-1]))

        # car 5 path
        self.x_car_5 = np.hstack((self.x5[::-1], self.x13[::-1], self.x3[::-1], self.x11[::-1], self.x2[::-1]))
        self.y_car_5 = np.hstack((self.y5[::-1], self.y13[::-1], self.y3[::-1], self.y11[::-1], self.y2[::-1]))

        # registering the vehicles
        self.pos_car_1 = Point(-4.0, 0.0, 0.0)
        self.yaw_car_1 = 0
        self.lin_vel_1 = 0.2
        self.ang_vel_1 = 0.0
        self.car_1_pose = Pose(self.pos_car_1, quaternion_from_euler(0, 0, self.yaw_car_1))
        self.car_1_twist = Twist(self.lin_vel_1, self.ang_vel_1)
        self.s_car_1 = 0 
        self.d_car_1 = 0.0

        self.pos_car_2 = Point(8.0, -2.0, 0.0)
        self.yaw_car_2 = 2.36
        self.lin_vel_2 = 0.1
        self.ang_vel_2 = 0.0
        self.car_2_pose = Pose(self.pos_car_2, quaternion_from_euler(0, 0, self.yaw_car_2))
        self.car_2_twist = Twist(self.lin_vel_2, self.ang_vel_2)
        self.s_car_2 = 0 
        self.d_car_2 = 0.0

        self.pos_car_3 = Point(0.5, 3.0, 0.0)
        self.yaw_car_3 = -1.57
        self.lin_vel_3 = 0.1
        self.ang_vel_3 = 0.0
        self.car_3_pose = Pose(self.pos_car_3, quaternion_from_euler(0, 0, self.yaw_car_3))
        self.car_3_twist = Twist(self.lin_vel_3, self.ang_vel_3)
        self.s_car_3 = 0 
        self.d_car_3 = 0.0

        self.pos_car_4 = Point(-6.0, -8.0, 0.0)
        self.yaw_car_4 = 1.57
        self.lin_vel_4 = 0.1
        self.ang_vel_4 = 0.0
        self.car_4_pose = Pose(self.pos_car_4, quaternion_from_euler(0, 0, self.yaw_car_4))
        self.car_4_twist = Twist(self.lin_vel_4, self.ang_vel_4)
        self.s_car_4 = 0 
        self.d_car_4 = 0.0

        self.pos_car_5 = Point(0.5, -3.0, 0.0)
        self.yaw_car_5 = 1.57
        self.lin_vel_5 = 0.2
        self.ang_vel_5 = 0.0
        self.car_5_pose = Pose(self.pos_car_5, quaternion_from_euler(0, 0, self.yaw_car_5))
        self.car_5_twist = Twist(self.lin_vel_5, self.ang_vel_5)
        self.s_car_5 = 0 
        self.d_car_5 = 0.0
        
        # past information storage initialization, later record it from the camera
        self.past_vel_1 = self.velocity(self.lin_vel_1, self.factor)
        self.past_d_1 = self.dist_from_center(self.d_car_1, self.factor)
        self.past_vel_2 = self.velocity(self.lin_vel_2, self.factor)
        self.past_d_2 = self.dist_from_center(self.d_car_2, self.factor)
        self.past_vel_3 = self.velocity(self.lin_vel_3, self.factor)
        self.past_d_3 = self.dist_from_center(self.d_car_3, self.factor)
        self.past_vel_4 = self.velocity(self.lin_vel_4, self.factor)
        self.past_d_4 = self.dist_from_center(self.d_car_4, self.factor)
        self.past_vel_5 = self.velocity(self.lin_vel_5, self.factor)
        self.past_d_5 = self.dist_from_center(self.d_car_5, self.factor)
        
        # future waypoints from current point
        self.future_waypoints_x_1, self.future_waypoints_y_1, _  = self.get_future_trajectory(self.x_car_1, self.y_car_1, [self.pos_car_1.x, self.pos_car_1.y], self.past_vel_1, self.ang_vel_1, self.past_d_1, self.pub1)
        self.future_waypoints_x_2, self.future_waypoints_y_2, _  = self.get_future_trajectory(self.x_car_2, self.y_car_2, [self.pos_car_2.x, self.pos_car_2.y], self.past_vel_2, self.ang_vel_2, self.past_d_2, self.pub2)
        self.future_waypoints_x_3, self.future_waypoints_y_3, _  = self.get_future_trajectory(self.x_car_3, self.y_car_3, [self.pos_car_3.x, self.pos_car_3.y], self.past_vel_3, self.ang_vel_3, self.past_d_3, self.pub3)
        self.future_waypoints_x_4, self.future_waypoints_y_4, _  = self.get_future_trajectory(self.x_car_4, self.y_car_4, [self.pos_car_4.x, self.pos_car_4.y], self.past_vel_4, self.ang_vel_4, self.past_d_4, self.pub4)
        self.future_waypoints_x_5, self.future_waypoints_y_5, _  = self.get_future_trajectory(self.x_car_5, self.y_car_5, [self.pos_car_5.x, self.pos_car_5.y], self.past_vel_5, self.ang_vel_5, self.past_d_5, self.pub5)

        # defining the vehicles
        self.car_1 = Vehicle(self.car_1_pose, self.car_1_twist, self.s_car_1, self.d_car_1, self.past_vel_1, self.past_d_1, self.future_waypoints_x_1, self.future_waypoints_y_1, "car1")
        self.car_2 = Vehicle(self.car_2_pose, self.car_2_twist, self.s_car_2, self.d_car_2, self.past_vel_2, self.past_d_2, self.future_waypoints_x_2, self.future_waypoints_y_2, "car2")
        self.car_3 = Vehicle(self.car_3_pose, self.car_3_twist, self.s_car_3, self.d_car_3, self.past_vel_3, self.past_d_3, self.future_waypoints_x_3, self.future_waypoints_y_3, "car3")
        self.car_4 = Vehicle(self.car_4_pose, self.car_4_twist, self.s_car_4, self.d_car_4, self.past_vel_4, self.past_d_4, self.future_waypoints_x_4, self.future_waypoints_y_4, "car4")
        self.car_5 = Vehicle(self.car_5_pose, self.car_5_twist, self.s_car_5, self.d_car_5, self.past_vel_5, self.past_d_5, self.future_waypoints_x_5, self.future_waypoints_y_5, "car5")

        # define car 1 as ego vehicle
        self.car_1.register_ego(self.car_1)

    # average velocity from history
    def velocity(self, vel, factor):
        vel_profile = []
        for _ in range(self.horizon):
            vel_profile.append(vel+np.random.random()/factor)         
        return vel_profile

    # average offset from history
    def dist_from_center(self, d, factor):
        d_profile = []
        for _ in range(self.horizon):
            d_profile.append(d+np.random.random()/(factor*10))          
        return d_profile

    # collision check
    def lineIntersection(self, traj_1_x, traj_1_y, traj_2_x, traj_2_y):
        intersect = Point2D(0,0)
        p0_x = traj_1_x[0]
        p0_y = traj_1_y[0]
        p1_x = traj_1_x[-1]
        p1_y = traj_1_y[-1]
        p2_x = traj_2_x[0]
        p2_y = traj_2_y[0]
        p3_x = traj_2_x[-1]
        p3_y = traj_2_y[-1]

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
            print("!!COLLISION AHEAD!!")
            print("Wait for the other vehicle to pass")
            return True
        
        return False  # no collision

    # ************************ CHANGE THIS FUNCTION *************************
    # movement function 
    def move(self, lin_vel, ang_vel):
        update = Twist()
        update.linear.x = lin_vel
        update.angular.z = ang_vel
        self.pub.publish(update)

    # adding on theta to get the yaw of the vehicle
    def get_frenet_with_theta(self, x, y, path, s_map):
        if path == None:
            print("Empty map. Cannot return Frenet coordinates")
            return 0.0, 0.0, 0.0

        ind_closest = closest_point_ind(path, x, y)

        # Determine the indices of the 2 closest points
        if ind_closest < len(path):
            # Check if we are at the end of the segment
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

            # Get the 2 points
            if use_previous:
                p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
                p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
                prev_idx = ind_closest - 1
            else:
                p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
                p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
                prev_idx = ind_closest

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

    # future waypoints
    def PredictTrajectoryVehicles(self, init_x, init_y, path, s_map, v, d):    # msg_vehicles -> traffic_msg/PredictionArray
        s, d_curr, yaw = self.get_frenet_with_theta(init_x, init_y, path, s_map)
        d = (d_curr + d) / 2                                                # average of all the deviations from center
        future_x = []
        future_y = []
        for t in range(self.np_m):
            if t < self.interp_back_path:
                d_val = d - ((t*d) / self.interp_back_path)
                new_x, new_y, _ = get_xy(s+v*self.dt_m*t, d_val, path, s_map)
            else:
                new_x, new_y, _ = get_xy(s+v*self.dt_m*t, 0, path, s_map)
            future_x.append(new_x)
            future_y.append(new_y)
        return future_x, future_y, yaw, d

    # getting the future trajectory
    def get_future_trajectory(self, x, y, current_waypoint, past_v, w, past_d, pub):
        # return the future trajectory of the vehicle
        v = np.mean(past_v)
        d = np.mean(past_d)
        lane_line_list, lane_s_map = self.get_lane_and_s_map(x, y)
        future_x, future_y, yaw, d = self.PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map, v, d)
        # curr = move(current_waypoint[0], current_waypoint[1], v, dt_m, lane_line_list)
        # move(pub, v, w)
        # pub.publish(move)
        # update these waypoints as ros messages -> geometry_msgs.pose.position
        # later provide this information on ros traffic messages
        

        # yaw = np.pi/4
        # orientation = tf.quaternion_from_euler(0, 0, yaw)      # add yaw parameter to function
        # orientation = Quaternion(out[0], out[1], out[2], out[3])
        # pose = geometry_msgs.Pose(car_1_pose, orientation)
        # linear = geometry_msgs.Vector3(v*math.cos(yaw), v*math.sin(yaw), 0)
        # angular = geometry_msgs.Vector3(0, 0, yaw)
        # twist = geometry_msgs.Twist(linear, angular)

        # future trajectory obtained from the current waypoint
        future_x.insert(0, current_waypoint[0])
        future_y.insert(0, current_waypoint[1])

        # Vehicle.vehicle_dynamics[id] = [pose, twist, s, d, past_v, past_d, [future_x, future_y]]  # update in vehicle class
        return future_x, future_y, d #pose, twist, d

    # get the s map and lane info
    def get_lane_and_s_map(self, x, y):
        pose_arr = []
        # x_g, y_g = get_spline(current_waypoint[0],destination_waypoint[0],current_waypoint[1],destination_waypoint[1],np.pi/2,np.pi/4)
        lane_route = []
        for i in range(len(x)):
            lane_route.append([x[i], y[i]])
        
        for i in range(len(lane_route)-1):
            point = Point2D(lane_route[i][0], lane_route[i][1])
            # replace this by actual yaw of the vehicle maybe
            yaw = math.atan2((lane_route[i+1][1]-lane_route[i][1]),(lane_route[i+1][0]-lane_route[i][0]))
            quat = quaternion_from_euler(0,0,yaw)
            # quat = Quaternion(out[0], out[1], out[2], out[3])
            poses = PoseStamped(Header, Pose(point, quat))
            pose_arr.append(poses)
        # adding the last point
        pose_arr.append(PoseStamped(Header, Pose(Point2D(lane_route[-1][0], lane_route[-1][1]), quat)))
        path_route = Path(Header, pose_arr)
        lane_line_list, lane_s_map = path_to_list(path_route)

        return lane_line_list, lane_s_map

    def callback(self, pos):
        while not rospy.is_shutdown():
            # update1 = Twist()
            # update1.linear.x = np.mean(self.past_vel_1)
            # update1.angular.z = self.ang_vel_1
            # self.pub1.publish(update1)

            # update2 = Twist()
            # update2.linear.x = np.mean(self.past_vel_2)
            # update2.angular.z = self.ang_vel_2
            # self.pub2.publish(update2)

            if not self.lineIntersection(self.future_waypoints_x_1, self.future_waypoints_y_1, self.future_waypoints_x_2, self.future_waypoints_y_2):
                self.car_2.register_vehicle(self.car_1, self.car_2)
                self.future_waypoints_x_1, self.future_waypoints_y_1, self.d_car_1 = self.get_future_trajectory(self.x_car_1, self.y_car_1, [self.pos_car_1.x, self.pos_car_1.y], self.past_vel_1, self.ang_vel_1, self.past_d_1, self.pub1)
                self.future_waypoints_x_2, self.future_waypoints_y_2, self.d_car_2 = self.get_future_trajectory(self.x_car_2, self.y_car_2, [self.pos_car_2.x, self.pos_car_2.y], self.past_vel_2, self.ang_vel_2, self.past_d_2, self.pub2) 
                
                self.car_1_pose = Pose(self.pos_car_1, quaternion_from_euler(0, 0, self.yaw_car_1))
                self.car_1_twist = Twist(np.mean(self.past_vel_1), self.ang_vel_1)
                self.car_2_pose = Pose(self.pos_car_2, quaternion_from_euler(0, 0, self.yaw_car_2))
                self.car_2_twist = Twist(np.mean(self.past_vel_2), self.ang_vel_2)
                self.car_1 = Vehicle(self.car_1_pose, self.car_1_twist, self.s_car_1, self.d_car_1, self.past_vel_1, self.past_d_1, self.future_waypoints_x_1, self.future_waypoints_y_1, "car1")
                self.car_2 = Vehicle(self.car_2_pose, self.car_2_twist, self.s_car_2, self.d_car_2, self.past_vel_2, self.past_d_2, self.future_waypoints_x_2, self.future_waypoints_y_2, "car2")
                # plt.plot(future_waypoints_x_1, future_waypoints_y_1, 'r--')
                # plt.plot(car_1_pose.position.x, car_1_pose.position.y, 'b')
                # plt.plot(future_waypoints_x_2, future_waypoints_y_2, 'g--')
                # plt.plot(car_2_pose.position.x, car_2_pose.position.y, 'b')
                # plt.pause(0.1)

            else:
                self.car_2.register_vehicle(self.car_1, self.car_2)
                # future_waypoints_x_2, future_waypoints_y_2, car_2_pose, car_2_twist, d_car_2 = get_future_trajectory(x_car_2, y_car_2, [pos_car_2.x, pos_car_2.y], past_vel_2, ang_vel_2, past_d_2, pub2)
                self.future_waypoints_x_2, self.future_waypoints_y_2, self.d_car_2 = self.get_future_trajectory(self.x_car_2, self.y_car_2, [self.pos_car_2.x, self.pos_car_2.y], self.past_vel_2, self.ang_vel_2, self.past_d_2, self.pub2) 
                self.car_2_pose = Pose(self.pos_car_2, quaternion_from_euler(0, 0, self.yaw_car_2))
                self.car_2_twist = Twist(np.mean(self.past_vel_2), self.ang_vel_2)
                self.car_2 = Vehicle(self.car_2_pose, self.car_2_twist, self.s_car_2, self.d_car_2, self.past_vel_2, self.past_d_2, self.future_waypoints_x_2, self.future_waypoints_y_2, "car2")
                # plt.plot(future_waypoints_x_2, future_waypoints_y_2, 'g--')
                # plt.plot(car_2_pose.position.x, car_2_pose.position.y, 'b')
                # plt.pause(0.1)
          
def main():
    rospy.init_node('predictor', anonymous=True)
    sub = Subscriber()
    rospy.spin()

# main function
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass