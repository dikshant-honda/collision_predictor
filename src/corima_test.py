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

class Subscriber:
    def __init__(self):
        # variables
        self.width = 2                                  # lane width
        self.interp_back_path = 40                      # interpolate back to path after this # of steps
        self.plan_t_m = 3                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 0.5                                  # tolerance value for proximity check
        self.vision_radius = 3                          # check only nearby cars
        self.intersection_vision = 2                    # check card arriving near intersection
        self.car_at_junction = {"X":[], "Y":[], "T":[]} # dictionary for storing the ids at the junction
        
        # subscribers
        self.car_1_sub = message_filters.Subscriber('/tb3_1/odom', Odometry)
        self.car_2_sub = message_filters.Subscriber('/tb3_2/odom', Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.car_1_sub, self.car_2_sub], 10, 0.1)
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
            future_waypoints.append(Point(new_x, new_y, 0.0))
        return future_waypoints, d

    # getting the future trajectory
    def get_future_trajectory(self, car): 
        v = np.mean(car.past_vel)
        d = np.mean(car.past_d)
        lane_line_list, lane_s_map = self.get_lane_and_s_map(car.car_route)
        future_waypoints, d = self.PredictTrajectoryVehicles(car.pose.pose.pose.position.x, car.pose.pose.pose.position.y, lane_line_list, lane_s_map, v, d)
        car.d = d
        return future_waypoints

    def publishers(self, car, move):
        if car.id == "car_1":
            pub1.publish(move)
        if car.id == "car_2":
            pub2.publish(move)

    def correct_angle(self, angle):
        if angle > np.pi:
            angle = angle - 2*np.pi
        elif angle < -np.pi:
            angle = angle + 2*np.pi
        
        return angle

    def dubins_update(self, car):
        if car not in env.vehicle_states:
            self.add(car)
        path, _  = self.get_lane_and_s_map(car.car_route)
        x_pos, y_pos = car.pose.pose.pose.position.x, car.pose.pose.pose.position.y
        ind_closest = closest_point_ind(path, x_pos, y_pos)        
        yaw_path = car.car_yaw
        # still on the lane
        if ind_closest < len(path)-1:
            x, y, z, w = car.pose.pose.pose.orientation.x, car.pose.pose.pose.orientation.y, car.pose.pose.pose.orientation.z, car.pose.pose.pose.orientation.w
            _, _, init_yaw = euler_from_quaternion([x, y, z, w])

            # PI controller for yaw correction
            pi = PI(P=0.5, I = 1000)
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
            self.EOL(car)

        # publish the move message
        self.publishers(car, move)

    '''
    def update(self, car):
        self.add(car)
        path, _  = self.get_lane_and_s_map(car.car_route)
        x, y = car.pose.pose.pose.position.x, car.pose.pose.pose.position.y
        ind_closest = closest_point_ind(path, x, y)

        # still on the lane
        if ind_closest < len(path)-1:
            p1, p2, _ = self.closest_points(ind_closest, path, x, y)
            d = (x - p1.x)*(p2.y - p1.y) - (y - p1.y)*(p2.x - p1.x)
            if d < 0:
                factor = -1
            elif d > 0:
                factor = 1
            else:
                factor = 0

            # PI controller 
            pi = PI()
            pi.SetPoint = [path[ind_closest].x, path[ind_closest].y]
            feedback = [x, y]
            pi.update(feedback)
            yaw = pi.output

            v = np.mean(car.past_vel) 

            # twist message to be published
            linear = Vector3(v, 0, 0)
            angular = Vector3(0, 0, yaw*factor)
            move = Twist(linear, angular)
            car.stop = False

        # stop after reaching the end of lane
        else:
            linear = Vector3(0, 0, 0)
            angular = Vector3(0, 0, 0)
            move = Twist(linear, angular)
            car.stop = True
            self.EOL(car)

        # publish the move message
        self.publishers(car, move)
        # update car data
        self.callbacks(car)
    '''

    # time synchronized callback
    def callback(self, veh_1, veh_2):
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

    def stop(self, car):
        print("!!! Stop:", car.id, "!!!")
        linear = Vector3(0, 0, 0)
        angular = Vector3(0, 0, 0)
        move = Twist(linear, angular)
        car.stop = True
        self.publishers(car, move)

    # collision check using line intersection technique
    def lineIntersection(self, future_waypoints_1, future_waypoints_2):
        intersect = Point()
        p0_x = future_waypoints_1[0].x
        p0_y = future_waypoints_1[0].y
        p1_x = future_waypoints_1[-1].x
        p1_y = future_waypoints_1[-1].y
        p2_x = future_waypoints_2[0].x
        p2_y = future_waypoints_2[0].y
        p3_x = future_waypoints_2[-1].x
        p3_y = future_waypoints_2[-1].y

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

    # collision check by vicinity or point-wise check
    def collision(self, points1, points2):
        for i in range(len(points1)):
            for j in range(len(points2)):
                if distance(points1[i].x, points1[i].y, points2[j].x, points2[j].y) < self.tol:
                    return True
        return False

    '''
    # computationally better way to test
    def collision(self, points1, points2):
        points2.sort()
        for point in points1:
            contenders = points2[bisect(points2, (point.x-self.tol-self.delta, 0)):bisect(points2,(point.x+self.tol+self.delta, 0))]
            contenders = list(map(lambda p:(p.y,p.x), contenders))
            contenders.sort()
            contenders = contenders[bisect(contenders,(point.y-self.tol-self.delta, 0)) : bisect(contenders,(point.y+self.tol+self.delta, 0))]
            matches = [(point, p2) for p2 in contenders if (point.x - p2.y)**2 + (point.y - p2.x)**2 <= self.tol**2]
            if len(matches) > 0:
                return True
        return False    
    '''

    def point_to_arr(self, car, points_arr):
        file_name = "traj_"+car+".txt"
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

    def update_env(self, env):
        # printing environment information
        self.at_intersection(car_1)
        self.at_intersection(car_2)
        # print(self.car_at_junction)
        print("current number of moving vehicles:", env.vehicles)

    def plot_current_position(self):
        car_1_pos = car_1.pose.pose.pose.position
        car_2_pos = car_2.pose.pose.pose.position

        plt.plot(car_1_pos.y, -car_1_pos.x, 'r*')
        plt.plot(car_2_pos.y, -car_2_pos.x, 'c*')

    def plot_future_trajectory(self, car1, car2):
        self.point_to_arr(car1.id, car1.future_waypoints)
        self.point_to_arr(car2.id, car2.future_waypoints)
        x1, y1 = plotter(car1.id)
        x2, y2 = plotter(car2.id)

        # plot trajectories
        plt.plot(y1, -x1, '-')
        plt.plot(y2, -x2, '-')

    def main(self):
        time_taken = 0
        arr_1_x = []
        arr_1_y = []
        arr_2_x = []
        arr_2_y = []
        while not rospy.is_shutdown():
            start = time.time()

            # print current position of the vehicle
            self.plot_current_position()

            car_1_pos = car_1.pose.pose.pose.position
            car_2_pos = car_2.pose.pose.pose.position

            arr_1_x.append(car_1_pos.x)
            arr_1_y.append(car_1_pos.y)
            arr_2_x.append(car_2_pos.x)
            arr_2_y.append(car_2_pos.y)

            # scenario #1
            self.dubins_update(car_1)
            self.dubins_update(car_2)

            # update environment
            # self.update_env(env)

            # for interaction between 2 vehicles only
            # later add interaction of more vehicles
            car_1.future_waypoints = self.get_future_trajectory(car_1)
            car_2.future_waypoints = self.get_future_trajectory(car_2)
            self.plot_future_trajectory(car_1, car_2)
            if self.collision(car_1.future_waypoints, car_2.future_waypoints):
                print("possibility of collision")
                self.stop(car_1)

            end = time.time()
            time_taken += end-start

            # plotting tools
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("Trajectories of the moving vehicles")
            plt.pause(0.000000001)

            print("Loop execution time", end-start)
            # print("time elapsed:", time_taken)
            print("**************")
            print("car1x:", arr_1_x)
            print("car1y:", arr_1_y)
            print("car2x:", arr_2_x)
            print("car2y:", arr_2_y)
            print("**************")
            print("------------------------------------------")
            if env.vehicles == 0:
                print("Execution Done")
                break   

if __name__ == '__main__':
    try:
        # registering the vehicles
        pos_car_1 = Point(-0.5, -10.0, 0.0)
        yaw_car_1 = 1.57
        v_1 = 1
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
        at_lane_1 = True
        at_junction_1 = False 

        pos_car_2 = Point(9.0, -0.5, 0.0)
        yaw_car_2 = 3.14
        v_2 = 1
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
        at_lane_2 = True
        at_junction_2 = False 

        # initialize the vehicles
        car_1 = VehicleState("car_1", car_1_odom, car_1_twist, past_vel_1, d_car_1, past_d_1, stop_1, future_waypoints_1, car_1_route, car_yaw_1, at_lane_1, at_junction_1)
        car_2 = VehicleState("car_2", car_2_odom, car_2_twist, past_vel_2, d_car_2, past_d_2, stop_2, future_waypoints_2, car_2_route, car_yaw_2, at_lane_2, at_junction_2)

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
        pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)

        sub = Subscriber()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass