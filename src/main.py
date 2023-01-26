#! /usr/bin/env python3

import rospy
import math
import time
import numpy as np
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist, Pose, PoseStamped, Vector3, PoseWithCovariance
from tf.transformations import quaternion_from_euler
from frenet import *
from geometry_utils import *
from lane_info import *
from collision_predictor.msg import Environment, VehicleState
import message_filters

class PI:
    def __init__(self, P = 0.97, I = 100000, current_time = None):
        self.Kp = P
        self.Ki = I

        self.sample_time = 0.1
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        self.SetPoint = [0.0, 0.0]
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.last_error = 0.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        error =  distance(feedback_value[0], feedback_value[1], self.SetPoint[0], self.SetPoint[1])
        
        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time <= self.sample_time:
            self.PTerm = self.Kp*delta_error
            self.ITerm += error*delta_time

            self.last_time = self.current_time
            self.last_error = error

        self.output = self.PTerm + self.Ki * self.ITerm 

class Subscriber:
    def __init__(self):
        # variables
        self.width = 2                                  # lane width
        self.interp_back_path = 10                      # interpolate back to path after this # of steps
        self.plan_t_m = 3                               # planning horizon
        self.dt_m = 0.1                                 # time step update
        self.np_m = int(self.plan_t_m/self.dt_m)        # number of future waypoints
        self.tol = 0.1                                  # tolerance value for proximity check
        self.vision_radius = 2                          # check only nearby cars

        # # time synchronized callback
        # ts.registerCallback(self.callback)
        
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
        # d = (d_curr + d) / 2                    # average of all the deviations from center
        d = d_curr
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

    # Vehicle state subcribers
    def callback1(self, msg):
        car_1.pose = msg
        car_1.twist = msg.twist.twist
        car_1.past_vel.pop(0)
        # v = np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        v = v_1
        car_1.past_vel.append(v)
        car_1.past_d.pop(0)
        car_1.past_d.append(car_1.d)

    def callback2(self, msg):
        car_2.pose = msg
        car_2.twist = msg.twist.twist
        car_2.past_vel.pop(0)
        v = v_2
        # v = np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        car_2.past_vel.append(v)
        car_2.past_d.pop(0)
        car_2.past_d.append(car_2.d) 

    def callback3(self, msg):
        car_3.pose = msg
        car_3.twist = msg.twist.twist
        car_3.past_vel.pop(0)
        v = v_3
        # v = np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        car_3.past_vel.append(v)
        car_3.past_d.pop(0)
        car_3.past_d.append(car_3.d)

    def callback4(self, msg):
        car_4.pose = msg
        car_4.twist = msg.twist.twist
        car_4.past_vel.pop(0)
        v = v_4
        # v = np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        car_4.past_vel.append(v)
        car_4.past_d.pop(0)
        car_4.past_d.append(car_4.d) 

    def callback5(self, msg):
        car_5.pose = msg
        car_5.twist = msg.twist.twist
        car_5.past_vel.pop(0)
        v = v_5
        # v = np.sqrt(msg.twist.twist.linear.x**2+msg.twist.twist.linear.y**2)
        car_5.past_vel.append(v)
        car_5.past_d.pop(0)
        car_5.past_d.append(car_5.d) 

    def callbacks(self, car):
        if car.id == "car_1":
            rospy.Subscriber('/tb3_1/odom', Odometry, self.callback1)
        if car.id == "car_2":
            rospy.Subscriber('/tb3_2/odom', Odometry, self.callback2)
        if car.id == "car_3":
            rospy.Subscriber('/tb3_3/odom', Odometry, self.callback3)
        if car.id == "car_4":
            rospy.Subscriber('/tb3_4/odom', Odometry, self.callback4)
        if car.id == "car_5":
            rospy.Subscriber('/tb3_5/odom', Odometry, self.callback5)

    def publishers(self, car, move):
        if car.id == "car_1":
            pub1.publish(move)
        if car.id == "car_2":
            pub2.publish(move)
        if car.id == "car_3":
            pub3.publish(move)
        if car.id == "car_4":
            pub4.publish(move)
        if car.id == "car_5":
            pub5.publish(move)

    def update(self, car):
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

        # publish the move message
        self.publishers(car, move)
        # update car data
        self.callbacks(car)

        # ------------------------------- not working properly ----------------------
        # time synchronized callback
        # def callback(veh_1, veh_2, veh_3, veh_4, veh_5):
        #     print("callback function called")
        #     # car 1 updates
        #     car_1.pose = veh_1
        #     car_1.twist = veh_1.twist.twist
        #     car_1.past_vel.pop(0)
        #     car_1.past_vel.append(v_1)
        #     car_1.past_d.pop(0)
        #     car_1.past_d.append(car_1.d)

        #     # car 2 updates
        #     car_2.pose = veh_2
        #     car_2.twist = veh_2.twist.twist
        #     car_2.past_vel.pop(0)
        #     car_2.past_vel.append(v_2)
        #     car_2.past_d.pop(0)
        #     car_2.past_d.append(car_2.d)

        #     # car 3 updates
        #     car_3.pose = veh_3
        #     car_3.twist = veh_3.twist.twist
        #     car_3.past_vel.pop(0)
        #     car_3.past_vel.append(v_3)
        #     car_3.past_d.pop(0)
        #     car_3.past_d.append(car_3.d)

        #     # car 4 updates
        #     car_4.pose = veh_4
        #     car_4.twist = veh_4.twist.twist
        #     car_4.past_vel.pop(0)
        #     car_4.past_vel.append(v_4)
        #     car_4.past_d.pop(0)
        #     car_4.past_d.append(car_4.d)

        #     # car 5 updates
        #     car_5.pose = veh_5
        #     car_5.twist = veh_5.twist.twist
        #     car_5.past_vel.pop(0)
        #     car_5.past_vel.append(v_5)
        #     car_5.past_d.pop(0)
        #     car_5.past_d.append(car_5.d)
            
        # ts = message_filters.TimeSynchronizer([car_1_sub, car_2_sub, car_3_sub, car_4_sub, car_5_sub], 1)
        # ts.registerCallback(callback)

    def stop(self, car):
        print("trying to stop:", car.id)
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
        env.deregister = True
        env.vehicles -= 1
        env.vehicle_states.remove(car)

    # joint functions
    # def update(self, car):
    #     self.update(car)

    # def EOL(self, car):
    #     self.removal(car)

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

    def point_to_arr(self, points_arr):
        arr_x = []
        arr_y = []
        for i in range(len(points_arr)):
            arr_x.append(points_arr[i].x)
            arr_y.append(points_arr[i].y)
        return arr_x, arr_y

    def inVicinity(self, car1, car2):
        car1_pos = car1.pose.pose.pose.position
        car2_pos = car2.pose.pose.pose.position
        if distance(car1_pos.x, car1_pos.y, car2_pos.x, car2_pos.y) < self.vision_radius:
            return True

    def main(self):
        self.add(car_1)
        self.add(car_5)
        while not rospy.is_shutdown():
            car_1.future_waypoints = self.get_future_trajectory(car_1)
            car_2.future_waypoints = self.get_future_trajectory(car_2)
            car_3.future_waypoints = self.get_future_trajectory(car_3)
            car_4.future_waypoints = self.get_future_trajectory(car_4)
            car_5.future_waypoints = self.get_future_trajectory(car_5)
            
            # self.update(car_1)
            # self.update(car_2)
            # self.update(car_3)
            # self.update(car_4)
            # self.update(car_5)
            if not self.collision(car_1.future_waypoints, car_2.future_waypoints):
                # arr_x, arr_y = self.point_to_arr(car_3.future_waypoints)
                # print(arr_x, arr_y)
                self.update(car_1)
                # print("------------------------------")
                # arr_x_, arr_y_ = self.point_to_arr(car_4.future_waypoints)
                # print(arr_x_, arr_y_)
                self.update(car_2)
                # print("******************************")
            else:
            # #     print(car_3.future_waypoints)
            # #     print("-------------")
            # #     print(car_4.future_waypoints)
            #     # print("*************")
                print("possibility of collision")
                self.stop(car_1)
                self.stop(car_2)
                break
            # if car_1.stop:
            #     self.removal(car_1)
            # if car_2.stop:
            #     self.removal(car_2)
            # if car_3.stop:
            #     self.removal(car_3)
            # if car_4.stop:
            #     self.removal(car_4)
            # if car_5.stop:
            #     self.removal(car_5)
            # if not car_1.future_waypoints and not car_2.future_waypoints:
            #     print("there's no future trajectory, vehicle has just started interacting with the environment")
            #     print("time to register the vehicle into the environment")
            #     # adding vehicles to the environment
            #     self.add(car_1)
            #     self.add(car_2)
            # else:
            #     if not self.lineIntersection(car_1.future_waypoints, car_2.future_waypoints):
            #         # start checking for intersection
            #         car_1.future_waypoints, car_1.d = self.get_future_trajectory(car_1)
            #         car_2.future_waypoints, car_2.d = self.get_future_trajectory(car_2)
            #         self.update(car_1)
            #         self.update(car_2)
            #         if car_1.stop:
            #             self.removal(car_1)
            #         if car_2.stop:
            #             self.removal(car_2)
                # add more functionalities
        rospy.sleep(1.0)

    def print_info(env):
        print("current number of vehicles:", env.vehicles)
        print("**************************************")        

if __name__ == '__main__':
    try:
        # registering the vehicles
        pos_car_1 = Point(-4.0, 0.0, 0.0)
        yaw_car_1 = 0
        v_1 = 0.4
        lin_vel_1 = Vector3(v_1, 0.0, 0.0)
        ang_vel_1 = Vector3(0.0, 0.0, 0.0)
        car_1_pose = Pose(pos_car_1, quaternion_from_euler(0, 0, yaw_car_1))
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

        pos_car_2 = Point(8.0, -2.0, 0.0)
        yaw_car_2 = 2.36
        v_2 = 0.3
        lin_vel_2 = Vector3(v_2, 0.0, 0.0)
        ang_vel_2 = Vector3(0.0, 0.0, 0.0)
        car_2_pose = Pose(pos_car_2, quaternion_from_euler(0, 0, yaw_car_2))
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

        pos_car_3 = Point(0.0, 3.0, 0.0)
        yaw_car_3 = -1.57
        v_3 = 0.3
        lin_vel_3 = Vector3(v_3, 0.0, 0.0)
        ang_vel_3 = Vector3(0.0, 0.0, 0.0)
        car_3_pose = Pose(pos_car_3, quaternion_from_euler(0, 0, yaw_car_3))
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

        pos_car_4 = Point(0.0, -3.0, 0.0)
        yaw_car_4 = 1.57
        v_4 = 0.3
        lin_vel_4 = Vector3(v_4, 0.0, 0.0)
        ang_vel_4 = Vector3(0.0, 0.0, 0.0)
        car_4_pose = Pose(pos_car_4, quaternion_from_euler(0, 0, yaw_car_4))
        car_4_twist = Twist(lin_vel_4, ang_vel_4)
        s_car_4 = 0 
        d_car_4 = 0.0
        past_vel_4 = [v_4]*10
        past_d_4 = [d_car_4]*10
        covariance_4 = [[0 for _ in range(6)] for _ in range(6)]
        car_4_pose_with_covariance = PoseWithCovariance(car_4_pose, covariance_4)
        car_4_odom = Odometry(Header, "base_footprint", car_4_pose_with_covariance, car_4_twist) 
        stop_4 = False  
        future_waypoints_4 = []

        pos_car_5 = Point(-6.3, -8.0, 0.0)
        yaw_car_5 = 1.57
        v_5 = 0.45
        lin_vel_5 = Vector3(v_5, 0.0, 0.0)
        ang_vel_5 = Vector3(0.0, 0.0, 0.0)
        car_5_pose = Pose(pos_car_5, quaternion_from_euler(0, 0, yaw_car_5))
        car_5_twist = Twist(lin_vel_5, ang_vel_5)
        s_car_5 = 0 
        d_car_5 = 0.0
        past_vel_5 = [v_5]*10
        past_d_5 = [d_car_5]*10
        covariance_5 = [[0 for _ in range(6)] for _ in range(6)]
        car_5_pose_with_covariance = PoseWithCovariance(car_5_pose, covariance_5)
        car_5_odom = Odometry(Header, "base_footprint", car_5_pose_with_covariance, car_5_twist) 
        stop_5 = False  
        future_waypoints_5 = []

        # initialize the vehicles
        car_1 = VehicleState("car_1", car_1_odom, car_1_twist, past_vel_1, d_car_1, past_d_1, stop_1, future_waypoints_1, car_1_route)
        car_2 = VehicleState("car_2", car_2_odom, car_2_twist, past_vel_2, d_car_2, past_d_2, stop_2, future_waypoints_2, car_2_route)
        car_3 = VehicleState("car_3", car_3_odom, car_3_twist, past_vel_3, d_car_3, past_d_3, stop_3, future_waypoints_3, car_3_route)
        car_4 = VehicleState("car_4", car_4_odom, car_4_twist, past_vel_4, d_car_4, past_d_4, stop_4, future_waypoints_4, car_4_route)
        car_5 = VehicleState("car_5", car_5_odom, car_5_twist, past_vel_5, d_car_5, past_d_5, stop_5, future_waypoints_5, car_5_route)

        # environment setup
        no_of_vehicles = 0
        vehicle_list = []
        at_junction = False
        register = False
        deregister = False 
        interaction = False
        env = Environment(no_of_vehicles, vehicle_list, at_junction, register, deregister, interaction)

        # text files for plotting the future trajectories of the vehicles
        # file3 = open("future_waypoints_car_3.txt", "w")

        rospy.init_node('predictor', anonymous=True)
        pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
        pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
        pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
        pub4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=10)
        pub5 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=10)
        
        # # subscribers
        # car_1_sub = message_filters.Subscriber('/tb3_1/odom', Odometry)
        # car_2_sub = message_filters.Subscriber('/tb3_2/odom', Odometry)
        # car_3_sub = message_filters.Subscriber('/tb3_3/odom', Odometry)
        # car_4_sub = message_filters.Subscriber('/tb3_4/odom', Odometry)
        # car_5_sub = message_filters.Subscriber('/tb3_5/odom', Odometry)

        sub = Subscriber()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass