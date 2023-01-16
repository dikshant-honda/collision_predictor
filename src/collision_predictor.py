#! /usr/bin/env python3

import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import std_msgs
import nav_msgs
import geometry_msgs 
import tf
from frenet import *
from geometry_utils import *
from lane_info import *

class Vehicle:
    count = 0
    ids = []
    vehicle_dynamics = {}        # [pose, twist, s, d, past, future]
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
        v = np.sqrt(twist.linear.x**2+twist.linear.y**2)
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
        if ego_vicinity(ego, veh):
            # print("start registering")
            self.add_vehicle(veh)


# check for the vehicles which are in the vicinity of the ego vehicle
def ego_vicinity(ego, veh):
    ego_pos = ego.pose.position
    veh_pos = veh.pose.position
    if distance(ego_pos.x, ego_pos.y, veh_pos.x, veh_pos.y) < vision_radius:
        return True

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

# check for the vehicles which are in the vicinity of the ego vehicle
def ego_vicinity(ego, veh):
    ego_pos = ego.pose.position
    veh_pos = veh.pose.position
    if distance(ego_pos.x, ego_pos.y, veh_pos.x, veh_pos.y) < vision_radius:
        return True

# collision check
def lineIntersection(traj_1_x, traj_1_y, traj_2_x, traj_2_y):
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

# # change this
# def move(x, y, v, dt_m, path):
#     # find the closest index from the curve and compute theta between those points
#     # shift the vehicle along that direction to get the modified points

#     ind_closest = closest_point_ind(path, x, y)
#     # Determine the indices of the 2 closest points
#     if ind_closest < len(path):
#         # Check if we are at the end of the segment
#         if ind_closest == len(path) - 1:
#             use_previous = True
#         elif ind_closest == 0:
#             use_previous = False
#         else:
#             dist_prev = distance(path[ind_closest-1].x, path[ind_closest-1].y, x, y)
#             dist_next = distance(path[ind_closest+1].x, path[ind_closest+1].y, x, y)

#             if dist_prev <= dist_next:
#                 use_previous = True
#             else:
#                 use_previous = False

#         # Get the 2 points
#         if use_previous:
#             p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
#             p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
#         else:
#             p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
#             p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)

#         # Get the point in the local coordinate with center p1
#         theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
#     new_x = x + v*np.cos(theta)*dt_m
#     new_y = y + v*np.sin(theta)*dt_m

#     return [new_x, new_y]

def move(pub, lin_vel, ang_vel):
    update = geometry_msgs.Twist()
    update.linear.x = lin_vel
    update.angular.z = ang_vel
    pub.publish(update)

# adding on theta to get the yaw of the vehicle
def get_frenet_with_theta(x, y, path, s_map):
    if path == None:
        print("Empty map. Cannot return Frenet coordinates")
        return 0.0, 0.0, 0.0, False

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
        return 0.0, 0.0, 0.0, False

    return p_s, p_d, theta, True

# future waypoints
def PredictTrajectoryVehicles(init_x, init_y, path, s_map, v, d):    # msg_vehicles -> traffic_msg/PredictionArray
    s, d_curr, yaw, _ = get_frenet_with_theta(init_x, init_y, path, s_map)
    d = (d_curr + d) / 2                                                # average of all the deviations from center
    future_x = []
    future_y = []
    for t in range(np_m):
        if t < interp_back_path:
            d_val = d - ((t*d) / interp_back_path)
            new_x, new_y, _ = get_xy(s+v*dt_m*t, d_val, path, s_map)
        else:
            new_x, new_y, _ = get_xy(s+v*dt_m*t, 0, path, s_map)
        future_x.append(new_x)
        future_y.append(new_y)
    return future_x, future_y, yaw, d

# getting the future trajectory
def get_future_trajectory(x, y, current_waypoint, past_v, w, past_d, pub):
    # return the future trajectory of the vehicle
    v = np.mean(past_v)
    d = np.mean(past_d)
    lane_line_list, lane_s_map = get_lane_and_s_map(x, y)
    future_x, future_y, yaw, d = PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map, v, d)
    # curr = move(current_waypoint[0], current_waypoint[1], v, dt_m, lane_line_list)
    move(pub, v, w)
    pub.publish(move)
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
def get_lane_and_s_map(x, y):
    pose_arr = []
    # x_g, y_g = get_spline(current_waypoint[0],destination_waypoint[0],current_waypoint[1],destination_waypoint[1],np.pi/2,np.pi/4)
    lane_route = []
    for i in range(len(x)):
        lane_route.append([x[i], y[i]])
    
    for i in range(len(lane_route)-1):
        point = geometry_msgs.Point(lane_route[i][0], lane_route[i][1])
        # replace this by actual yaw of the vehicle maybe
        yaw = math.atan2((lane_route[i+1][1]-lane_route[i][1]),(lane_route[i+1][0]-lane_route[i][0]))
        quat = tf.quaternion_from_euler(0,0,yaw)
        # quat = Quaternion(out[0], out[1], out[2], out[3])
        poses = geometry_msgs.PoseStamped(std_msgs.Header(), geometry_msgs.Pose(point, quat))
        pose_arr.append(poses)
    # adding the last point
    pose_arr.append(geometry_msgs.PoseStamped(std_msgs.Header(), geometry_msgs.Pose(geometry_msgs.Point(lane_route[-1][0], lane_route[-1][1]), quat)))
    path_route = Path(std_msgs.Header(), pose_arr)
    lane_line_list, lane_s_map = path_to_list(path_route)

    return lane_line_list, lane_s_map

# ros callbacks
def callback1(msg):
    global pos_car_1
    pos_car_1.x = msg.pose.pose.position.x
    pos_car_1.y = msg.pose.pose.position.y
    pos_car_1.z = msg.pose.pose.position.z

def callback2(msg):
    global pos_car_2
    pos_car_2.x = msg.pose.pose.position.x
    pos_car_2.y = msg.pose.pose.position.y
    pos_car_2.z = msg.pose.pose.position.z

# main function
if __name__ == '__main__':
    width = 2                       # lane width
    interp_back_path = 15           # interpolate back to path after this # of steps
    plan_t_m = 1                    # planning horizon
    dt_m = 0.1                      # time step update
    np_m = int(plan_t_m/dt_m)       # number of future waypoints
    vision_radius = 1               # register vehicles which are within this radius of the ego vehicle
    horizon = 10                    # number of past points info
    factor = 10                     # scale down factor for randomness

    rospy.init_node("collision detector") 

    global pos_car_1, pos_car_2, pos_car_3, pos_car_4, pos_car_5

    # velocity publishers
    pub1 = rospy.Publisher('/tb3_1/cmd_vel', geometry_msgs.Twist, queue_size=10)
    pub2 = rospy.Publisher('/tb3_2/cmd_vel', geometry_msgs.Twist, queue_size=10)
    pub3 = rospy.Publisher('/tb3_3/cmd_vel', geometry_msgs.Twist, queue_size=10)
    pub4 = rospy.Publisher('/tb3_4/cmd_vel', geometry_msgs.Twist, queue_size=10)
    pub5 = rospy.Publisher('/tb3_5/cmd_vel', geometry_msgs.Twist, queue_size=10)
    rate = rospy.Rate(2)

    # registering the vehicles
    pos_car_1 = geometry_msgs.Point(-4.0, 0.0, 0.0)
    yaw_car_1 = 0
    lin_vel_1 = 0.2
    ang_vel_1 = 0.0
    car_1_pose = geometry_msgs.Pose(pos_car_1, tf.quaternion_from_euler(0, 0, yaw_car_1))
    car_1_twist = geometry_msgs.Twist(lin_vel_1, ang_vel_1)
    s_car_1 = 0 
    d_car_1 = 0.0

    pos_car_2 = geometry_msgs.Point(8.0, -2.0, 0.0)
    yaw_car_2 = 2.36
    lin_vel_2 = 0.1
    ang_vel_2 = 0.0
    car_2_pose = geometry_msgs.Pose(pos_car_2, tf.quaternion_from_euler(0, 0, yaw_car_2))
    car_2_twist = geometry_msgs.Twist(lin_vel_2, ang_vel_2)
    s_car_2 = 0 
    d_car_2 = 0.0

    pos_car_3 = geometry_msgs.Point(0.5, 3.0, 0.0)
    yaw_car_3 = -1.57
    lin_vel_3 = 0.1
    ang_vel_3 = 0.0
    car_3_pose = geometry_msgs.Pose(pos_car_3, tf.quaternion_from_euler(0, 0, yaw_car_3))
    car_3_twist = geometry_msgs.Twist(lin_vel_3, ang_vel_3)
    s_car_3 = 0 
    d_car_3 = 0.0

    pos_car_4 = geometry_msgs.Point(-6.0, -8.0, 0.0)
    yaw_car_4 = 1.57
    lin_vel_4 = 0.1
    ang_vel_4 = 0.0
    car_4_pose = geometry_msgs.Pose(pos_car_4, tf.quaternion_from_euler(0, 0, yaw_car_4))
    car_4_twist = geometry_msgs.Twist(lin_vel_4, ang_vel_4)
    s_car_4 = 0 
    d_car_4 = 0.0

    pos_car_5 = geometry_msgs.Point(0.5, -3.0, 0.0)
    yaw_car_5 = 1.57
    lin_vel_5 = 0.2
    ang_vel_5 = 0.0
    car_5_pose = geometry_msgs.Pose(pos_car_5, tf.quaternion_from_euler(0, 0, yaw_car_5))
    car_5_twist = geometry_msgs.Twist(lin_vel_5, ang_vel_5)
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

    # future waypoints from current point
    future_waypoints_x_1, future_waypoints_y_1, _, _, _  = get_future_trajectory(x_car_1, y_car_1, car_1_pose.x, car_1_pose.y, past_vel_1, ang_vel_1, past_d_1, pub1)
    future_waypoints_x_2, future_waypoints_y_2, _, _, _  = get_future_trajectory(x_car_2, y_car_2, car_2_pose.x, car_2_pose.y, past_vel_2, ang_vel_2, past_d_2, pub2)
    future_waypoints_x_3, future_waypoints_y_3, _, _, _  = get_future_trajectory(x_car_3, y_car_3, car_3_pose.x, car_3_pose.y, past_vel_3, ang_vel_3, past_d_3, pub3)
    future_waypoints_x_4, future_waypoints_y_4, _, _, _  = get_future_trajectory(x_car_4, y_car_4, car_4_pose.x, car_4_pose.y, past_vel_4, ang_vel_4, past_d_4, pub4)
    future_waypoints_x_5, future_waypoints_y_5, _, _, _  = get_future_trajectory(x_car_5, y_car_5, car_5_pose.x, car_5_pose.y, past_vel_5, ang_vel_5, past_d_5, pub5)

    # defining the vehicles
    car_1 = Vehicle(car_1_pose, car_1_twist, s_car_1, d_car_1, past_vel_1, past_d_1, future_waypoints_x_1, future_waypoints_y_1, "car1")
    car_2 = Vehicle(car_2_pose, car_2_twist, s_car_2, d_car_2, past_vel_2, past_d_2, future_waypoints_x_2, future_waypoints_y_2, "car2")
    car_3 = Vehicle(car_3_pose, car_3_twist, s_car_3, d_car_3, past_vel_3, past_d_3, future_waypoints_x_3, future_waypoints_y_3, "car3")
    car_4 = Vehicle(car_4_pose, car_4_twist, s_car_4, d_car_4, past_vel_4, past_d_4, future_waypoints_x_4, future_waypoints_y_4, "car4")
    car_5 = Vehicle(car_5_pose, car_5_twist, s_car_5, d_car_5, past_vel_5, past_d_5, future_waypoints_x_5, future_waypoints_y_5, "car5")

    # define car 1 as ego vehicle
    car_1.register_ego(car_1)

    while not rospy.is_shutdown():
        # position subscribers
        sub1 = rospy.Subscriber("/tb3_1/odom", nav_msgs.Odometry, callback1)
        sub2 = rospy.Subscriber("/tb3_2/odom", nav_msgs.Odometry, callback2)
        # sub3 = rospy.Subscriber("/tb3_3/odom", nav_msgs.Odometry, callback3)
        # sub4 = rospy.Subscriber("/tb3_4/odom", nav_msgs.Odometry, callback4)
        # sub5 = rospy.Subscriber("/tb3_5/odom", nav_msgs.Odometry, callback5)

        if not lineIntersection(future_waypoints_x_1, future_waypoints_y_1, future_waypoints_x_2, future_waypoints_y_2):
            car_2.register_vehicle(car_1, car_2)
            # future_waypoints_x_1, future_waypoints_y_1, car_1_pose, car_1_twist, d_car_1 = get_future_trajectory(x_car_1, y_car_1, [pos_car_1.x, pos_car_1.y], past_vel_1, ang_vel_1, past_d_1, pub1)
            # future_waypoints_x_2, future_waypoints_y_2, car_2_pose, car_2_twist, d_car_2 = get_future_trajectory(x_car_2, y_car_2, [pos_car_2.x, pos_car_2.y], past_vel_2, ang_vel_2, past_d_2, pub2)
            future_waypoints_x_1, future_waypoints_y_1, d_car_1 = get_future_trajectory(x_car_1, y_car_1, [pos_car_1.x, pos_car_1.y], past_vel_1, ang_vel_1, past_d_1, pub1)
            future_waypoints_x_2, future_waypoints_y_2, d_car_2 = get_future_trajectory(x_car_2, y_car_2, [pos_car_2.x, pos_car_2.y], past_vel_2, ang_vel_2, past_d_2, pub2) 
            car_1_pose = geometry_msgs.Pose(pos_car_1, tf.quaternion_from_euler(0, 0, yaw_car_1))
            car_1_twist = geometry_msgs.Twist(np.mean(past_vel_1), ang_vel_1)
            car_2_pose = geometry_msgs.Pose(pos_car_2, tf.quaternion_from_euler(0, 0, yaw_car_2))
            car_2_twist = geometry_msgs.Twist(np.mean(past_vel_2), ang_vel_2)
            car_1 = Vehicle(car_1_pose, car_1_twist, s_car_1, d_car_1, past_vel_1, past_d_1, future_waypoints_x_1, future_waypoints_y_1, "car1")
            car_2 = Vehicle(car_2_pose, car_2_twist, s_car_2, d_car_2, past_vel_2, past_d_2, future_waypoints_x_2, future_waypoints_y_2, "car2")
            plt.plot(future_waypoints_x_1, future_waypoints_y_1, 'r--')
            plt.plot(car_1_pose.position.x, car_1_pose.position.y, 'b')
            plt.plot(future_waypoints_x_2, future_waypoints_y_2, 'g--')
            plt.plot(car_2_pose.position.x, car_2_pose.position.y, 'b')
            plt.pause(0.1)

        else:
            car_2.register_vehicle(car_1, car_2)
            future_waypoints_x_2, future_waypoints_y_2, car_2_pose, car_2_twist, d_car_2 = get_future_trajectory(x_car_2, y_car_2, [pos_car_2.x, pos_car_2.y], past_vel_2, ang_vel_2, past_d_2, pub2)
            car_2 = Vehicle(car_2_pose, car_2_twist, s_car_2, d_car_2, past_vel_2, past_d_2, future_waypoints_x_2, future_waypoints_y_2, "car2")
            plt.plot(future_waypoints_x_2, future_waypoints_y_2, 'g--')
            plt.plot(car_2_pose.position.x, car_2_pose.position.y, 'b')
            plt.pause(0.1)

        rate.sleep()
        rospy.spin()