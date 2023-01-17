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
    def __init__(self):
        # position subcribers
        self.sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.callback1)
        self.sub2 = rospy.Subscriber('/tb3_2/odom', Odometry, self.callback2)
        self.sub3 = rospy.Subscriber('/tb3_3/odom', Odometry, self.callback3)
        self.sub4 = rospy.Subscriber('/tb3_4/odom', Odometry, self.callback4)
        self.sub5 = rospy.Subscriber('/tb3_5/odom', Odometry, self.callback5)

        # variables
        self.width = 2                                  # lane width
        self.interp_back_path = 15                      # interpolate back to path after this # of steps
        self.plan_t_m = 5                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints

        self.car_1_pose = car_1_pose
        self.car_1_position = car_1_pose.position
        self.car_1_orientation = car_1_pose.orientation

        self.car_2_pose = car_2_pose
        self.car_2_position = car_2_pose.position
        self.car_2_orientation = car_2_pose.orientation

        # future waypoints from current point
        self.future_waypoints_x_1, self.future_waypoints_y_1, _  = self.get_future_trajectory(x_car_1, y_car_1, [pos_car_1.x, pos_car_1.y], past_vel_1, ang_vel_1, past_d_1, pub1)
        self.future_waypoints_x_2, self.future_waypoints_y_2, _  = self.get_future_trajectory(x_car_2, y_car_2, [pos_car_2.x, pos_car_2.y], past_vel_2, ang_vel_2, past_d_2, pub2)
        self.future_waypoints_x_3, self.future_waypoints_y_3, _  = self.get_future_trajectory(x_car_3, y_car_3, [pos_car_3.x, pos_car_3.y], past_vel_3, ang_vel_3, past_d_3, pub3)
        self.future_waypoints_x_4, self.future_waypoints_y_4, _  = self.get_future_trajectory(x_car_4, y_car_4, [pos_car_4.x, pos_car_4.y], past_vel_4, ang_vel_4, past_d_4, pub4)
        self.future_waypoints_x_5, self.future_waypoints_y_5, _  = self.get_future_trajectory(x_car_5, y_car_5, [pos_car_5.x, pos_car_5.y], past_vel_5, ang_vel_5, past_d_5, pub5)

        # # defining the vehicles
        # self.car_1 = Vehicle(car_1_pose, car_1_twist, s_car_1, d_car_1, past_vel_1, past_d_1, self.future_waypoints_x_1, self.future_waypoints_y_1, "car1")
        # self.car_2 = Vehicle(car_2_pose, car_2_twist, s_car_2, d_car_2, past_vel_2, past_d_2, self.future_waypoints_x_2, self.future_waypoints_y_2, "car2")
        # self.car_3 = Vehicle(car_3_pose, car_3_twist, s_car_3, d_car_3, past_vel_3, past_d_3, self.future_waypoints_x_3, self.future_waypoints_y_3, "car3")
        # self.car_4 = Vehicle(car_4_pose, car_4_twist, s_car_4, d_car_4, past_vel_4, past_d_4, self.future_waypoints_x_4, self.future_waypoints_y_4, "car4")
        # self.car_5 = Vehicle(car_5_pose, car_5_twist, s_car_5, d_car_5, past_vel_5, past_d_5, self.future_waypoints_x_5, self.future_waypoints_y_5, "car5")

        # # define car 1 as ego vehicle
        # self.car_1.register_ego(self.car_1)
        self.main()


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

            move1 = Twist()
            move1.linear.x = 0
            move1.angular.z = 0
            pub1.publish(move1)

            return True
        
        return False  # no collision

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
        # twist = geomelinear = geometry_msgs.Vector3(v*math.cos(yaw), v*math.sin(yaw), 0)
        # angular = geometry_msgs.Vector3(0, 0, yaw)
        # twist = geometry_msgs.Twist(linear, angular)try_msgs.Twist(linear, angular)

        # future trajectory obtained from the current waypoint
        future_x.insert(0, current_waypoint[0])
        future_y.insert(0, current_waypoint[1])

        return future_x, future_y, d #pose, twist, d

    def callback1(self, msg):
        self.car_1_pose = msg.pose.pose
        self.car_1_position = msg.pose.pose.position
        self.car_1_orientation = msg.pose.pose.orientation

    def callback2(self, msg):
        self.car_2_pose = msg.pose.pose
        self.car_2_position = msg.pose.pose.position
        self.car_2_orientation = msg.pose.pose.orientation

    def callback3(self, msg):
        self.car_3_pose = msg.pose.pose
        self.car_3_position = msg.pose.pose.position
        self.car_3_orientation = msg.pose.pose.orientation
    
    def callback4(self, msg):
        self.car_4_pose = msg.pose.pose
        self.car_4_position = msg.pose.pose.position
        self.car_4_orientation = msg.pose.pose.orientation

    def callback5(self, msg):
        self.car_5_pose = msg.pose.pose
        self.car_5_position = msg.pose.pose.position
        self.car_5_orientation = msg.pose.pose.orientation

    def update(self, lane_x, lane_y, x, y, vel):
        path, _ = self.get_lane_and_s_map(lane_x, lane_y)
        ind_closest = closest_point_ind(path, x, y)
        if ind_closest < len(path):
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
            else:
                p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
                p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
                p3 = Point2D(path[ind_closest + 2].x, path[ind_closest + 2].y)

            yaw = math.atan2(p2.y - p1.y, p2.x - p1.x)
            next_yaw = math.atan2(p3.y - p2.y, p3.x - p2.x)

        linear = Vector3(vel*math.cos(yaw), vel*math.sin(yaw), 0)
        # test
        print(next_yaw-yaw)
        # if next_yaw - yaw < 0.001:
        #     angular = Vector3(0, 0, 0)
        # else:
        angular = Vector3(0, 0, 3*(next_yaw - yaw)/self.dt_m)
        move = Twist(linear, angular)
        pub1.publish(move)
        # pub2.publish(move)
        return yaw


    def main(self):
        while not rospy.is_shutdown():
            if not self.lineIntersection(self.future_waypoints_x_1, self.future_waypoints_y_1, self.future_waypoints_x_2, self.future_waypoints_y_2):
            # self.car_2.register_vehicle(self.car_1, self.car_2)
                self.future_waypoints_x_1, self.future_waypoints_y_1, d_car_1 = self.get_future_trajectory(x_car_1, y_car_1, [self.car_1_position.x, self.car_1_position.y], past_vel_1, ang_vel_1, past_d_1, pub1)
                self.future_waypoints_x_2, self.future_waypoints_y_2, d_car_2 = self.get_future_trajectory(x_car_2, y_car_2, [self.car_2_position.x, self.car_2_position.y], past_vel_2, ang_vel_2, past_d_2, pub2) 
                
                # self.car_1_pose = Pose(self.car_1_pose, quaternion_from_euler(0, 0, self.yaw_car_1))
                self.car_1_twist = Twist(np.mean(past_vel_1), ang_vel_1)
                # self.car_2_pose = Pose(self.car_2_pose, quaternion_from_euler(0, 0, self.yaw_car_2))
                self.car_2_twist = Twist(np.mean(past_vel_2), ang_vel_2)
                # self.car_1 = Vehicle(self.car_1_pose, self.car_1_twist, self.s_car_1, self.d_car_1, self.past_vel_1, self.past_d_1, self.future_waypoints_x_1, self.future_waypoints_y_1, "car1")
                # self.car_2 = Vehicle(self.car_2_pose, self.car_2_twist, self.s_car_2, self.d_car_2, self.past_vel_2, self.past_d_2, self.future_waypoints_x_2, self.future_waypoints_y_2, "car2")

                # move1 = Twist()
                # move1.linear.x = self.car_1_twist.linear
                # move1.angular.z = ang_vel_1
                # pub1.publish(move1)
                self.update(x_car_1, y_car_1, self.car_1_position.x, self.car_1_position.y, lin_vel_1)
                # self.update(x_car_2, y_car_2, self.car_2_position.x, self.car_2_position.y, lin_vel_2)

                # move2 = Twist()
                # move2.linear.x = self.car_2_twist.linear
                # move2.angular.z = ang_vel_2
                # pub2.publish(move2)

                # plt.plot(self.future_waypoints_x_1, self.future_waypoints_y_1, 'r--')
                # plt.plot(self.car_1_pose.position.x, self.car_1_pose.position.y, 'b')
                # plt.plot(self.future_waypoints_x_2, self.future_waypoints_y_2, 'g--')
                # plt.plot(self.car_2_pose.position.x, self.car_2_pose.position.y, 'b')
                # plt.pause(0.1)
                
                # print(self.future_waypoints_x_1[-1], self.future_waypoints_y_1)
                # print(self.future_waypoints_x_2[-1], self.future_waypoints_y_2)
                # print()

            # else:
            #     self.car_2.register_vehicle(self.car_1, self.car_2)
            #     # future_waypoints_x_2, future_waypoints_y_2, car_2_pose, car_2_twist, d_car_2 = get_future_trajectory(x_car_2, y_car_2, [pos_car_2.x, pos_car_2.y], past_vel_2, ang_vel_2, past_d_2, pub2)
            #     self.future_waypoints_x_2, self.future_waypoints_y_2, self.d_car_2 = self.get_future_trajectory(self.x_car_2, self.y_car_2, [self.pos_car_2.x, self.pos_car_2.y], self.past_vel_2, self.ang_vel_2, self.past_d_2, pub2) 
            #     self.car_2_pose = Pose(self.pos_car_2, quaternion_from_euler(0, 0, self.yaw_car_2))
            #     self.car_2_twist = Twist(np.mean(self.past_vel_2), self.ang_vel_2)
            #     self.car_2 = Vehicle(self.car_2_pose, self.car_2_twist, self.s_car_2, self.d_car_2, self.past_vel_2, self.past_d_2, self.future_waypoints_x_2, self.future_waypoints_y_2, "car2")
                
            #     move2 = Twist()
            #     move2.linear.x = self.car_2_twist.linear
            #     move2.angular.z = self.ang_vel_2
            #     pub2.publish(move2)
                
                # plt.plot(self.uture_waypoints_x_2, self.future_waypoints_y_2, 'g--')
                # plt.plot(self.car_2_pose.position.x, self.car_2_pose.position.y, 'b')
                # plt.pause(0.1)
            
        rospy.sleep(1.0)

# average velocity from history
def velocity(vel, factor):
    vel_profile = []
    for _ in range(horizon):
        vel_profile.append(vel+np.random.random()/factor)         
    return vel_profile

# average offset from history
def dist_from_center(d, factor):
    d_profile = []
    for _ in range(horizon):
        d_profile.append(d+np.random.random()/(factor*10))          
    return d_profile

# main function
if __name__ == '__main__':
    try:
        horizon = 10                               # number of past points info
        factor = 10                                # scale down factor for randomness

        #  straight lanes
        x1, y1 = get_lanes([-6,-1],[-6,-8])     # down right
        x2, y2 = get_lanes([-6,8], [-6,1])      # down left
        x3, y3 = get_lanes([-5,0], [-1,0])      # down center
        x4, y4 = get_lanes([0,5], [0,1])        # center left
        x5, y5 = get_lanes([0,-1], [0,-5])       # center right
        x6, y6 = get_lanes([1,0], [5.5,0])      # up center
        x7, y7 = get_lanes([6.5,-0.5], [10,-4]) # up right
        x8, y8 = get_lanes([6.5,0.5], [10,4])   # up left

        # splines
        # T intersection
        x9, y9 = get_spline([-6,-1], [-5,0], np.pi/2,0)
        x10, y10 = get_spline([-6,-1], [-6,1], np.pi/2, -np.pi/2)
        x11, y11 = get_spline([-6,1], [-5,0], -np.pi/2, 0)

        # X intersection
        x12, y12 = get_spline([-1,0], [0,1], 0, np.pi/2)
        x13, y13 = get_spline([-1,0], [0,-1], 0, -np.pi/2)
        x14, y14 = get_spline([-1,0], [1,0], 0, 0)
        x15, y15 = get_spline([0,-1], [1,0], np.pi/2, 0)
        x16, y16 = get_spline([0,1], [0,-1], -np.pi/2, -np.pi/2)
        x17, y17 = get_spline([0,1], [1,0], -np.pi/2, 0)

        # Y intersection
        x18, y18 = get_spline([5.5,0], [6.5, 0.5], 0, np.pi/4)
        x19, y19 = get_spline([5.5,0], [6.5,-0.5], 0, -np.pi/4)
        x20, y20 = get_spline([6.5,0.5], [6.5,-0.5], -(np.pi/4+np.pi/2), -np.pi/4)

        # vehicle trajectories
        # car 1 path
        x_car_1 = np.hstack((x3, x14, x6, x18, x8))
        y_car_1 = np.hstack((y3, y14, y6, y18, y8))   

        # car 2 path 
        x_car_2 = np.hstack((x7[::-1], x19[::-1], x6[::-1], x14[::-1], x3[::-1]))
        y_car_2 = np.hstack((y7[::-1], y19[::-1], y6[::-1], y14[::-1], y3[::-1]))

        # car 3 path
        x_car_3 = np.hstack((x4, x17, x6, x18, x8))
        y_car_3 = np.hstack((y4, y17, y6, y18, y8))

        # car 4 path
        x_car_4 = np.hstack((x1[::-1], x9, x3, x12, x4[::-1]))
        y_car_4 = np.hstack((y1[::-1], y9, y3, y12, y4[::-1]))

        # car 5 path
        x_car_5 = np.hstack((x5[::-1], x13[::-1], x3[::-1], x11[::-1], x2[::-1]))
        y_car_5 = np.hstack((y5[::-1], y13[::-1], y3[::-1], y11[::-1], y2[::-1]))


        # registering the vehicles
        pos_car_1 = Point(-4.0, 0.0, 0.0)
        yaw_car_1 = 0
        lin_vel_1 = 0.4
        ang_vel_1 = 0.0
        car_1_pose = Pose(pos_car_1, quaternion_from_euler(0, 0, yaw_car_1))
        car_1_twist = Twist(lin_vel_1, ang_vel_1)
        s_car_1 = 0 
        d_car_1 = 0.0

        pos_car_2 = Point(8.0, -2.0, 0.0)
        yaw_car_2 = 2.36
        lin_vel_2 = 0.1
        ang_vel_2 = 0.0
        car_2_pose = Pose(pos_car_2, quaternion_from_euler(0, 0, yaw_car_2))
        car_2_twist = Twist(lin_vel_2, ang_vel_2)
        s_car_2 = 0 
        d_car_2 = 0.0

        pos_car_3 = Point(0.5, 3.0, 0.0)
        yaw_car_3 = -1.57
        lin_vel_3 = 0.1
        ang_vel_3 = 0.0
        car_3_pose = Pose(pos_car_3, quaternion_from_euler(0, 0, yaw_car_3))
        car_3_twist = Twist(lin_vel_3, ang_vel_3)
        s_car_3 = 0 
        d_car_3 = 0.0

        pos_car_4 = Point(-6.0, -8.0, 0.0)
        yaw_car_4 = 1.57
        lin_vel_4 = 0.1
        ang_vel_4 = 0.0
        car_4_pose = Pose(pos_car_4, quaternion_from_euler(0, 0, yaw_car_4))
        car_4_twist = Twist(lin_vel_4, ang_vel_4)
        s_car_4 = 0 
        d_car_4 = 0.0

        pos_car_5 = Point(0.5, -3.0, 0.0)
        yaw_car_5 = 1.57
        lin_vel_5 = 0.2
        ang_vel_5 = 0.0
        car_5_pose = Pose(pos_car_5, quaternion_from_euler(0, 0, yaw_car_5))
        car_5_twist = Twist(lin_vel_5, ang_vel_5)
        s_car_5 = 0 
        d_car_5 = 0.0

        # past information storage initialization, later record it from the camera
        past_vel_1 = velocity(lin_vel_1, factor)
        past_d_1 = dist_from_center(d_car_1, factor)
        past_vel_2 = velocity(lin_vel_2, factor)
        past_d_2 = dist_from_center(d_car_2, factor)
        past_vel_3 = velocity(lin_vel_3, factor)
        past_d_3 = dist_from_center(d_car_3, factor)
        past_vel_4 = velocity(lin_vel_4, factor)
        past_d_4 = dist_from_center(d_car_4, factor)
        past_vel_5 = velocity(lin_vel_5, factor)
        past_d_5 = dist_from_center(d_car_5, factor)

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