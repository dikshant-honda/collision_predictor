#! /usr/bin/env python3

import rospy
import numpy as np
import math
import bisect
import itertools
from typing import List

from geometry_msgs.msg import Point
from multi_vehicle_tracking.msg import queue
from collision_predictor.msg import vehicle
from visualization_msgs.msg import Marker


class Predictions:
    def __init__(self) -> None:
        """
        prediction class

        parameters:
            sub: get the environment information from the perception system
            lanes: lane information from the environment
            time_steps: time horizon over which we want to predict the future trajectory
            max_collision_allowance: maximum overlap area to consider as collision
            vehicle_size: vehicle size to check the uncertainity (can be added to vehicle class directly)
        """

        self.sub = rospy.Subscriber('env_info', queue, self.callback)
        self.lanes = Lanes()
        self.time_steps = 50
        self.max_collision_allowance = 0.1
        self.vehicle_size = 5

    def callback(self, data):
        """
        callback for accessing the ROS data from the perception

        agrs:
            data: data received from perception
        """

        self.env = data
        self.get_vehicle_data(self.env)

    def get_vehicle_data(self, env: queue) -> queue:
        """
        function to get the data of every vehicle

        args:
            env: environment information
        """

        for vehicle_info in env.info:
            vehicle_info = self.get_future_trajectory(vehicle_info)
        self.collision_alert(env)
        env_pub.publish(env)

    def get_future_trajectory(self, vehicle_info: vehicle) -> vehicle:
        """
        function to get the future trajectory of the vehicle and add noise to the predictions

        args:
            vehicle_info: individual vehicle data 
        """

        vehicle_info.future_trajectory = self.update(self.time_steps,
                                                     vehicle_info.x_position.data, vehicle_info.y_position.data, vehicle_info.direction.data, vehicle_info.speed.data)
        self.mark(vehicle_info.id, vehicle_info.direction,
                  vehicle_info.future_trajectory)
        self.true_predictions = self.add_noise(
            self.time_steps, vehicle_info.future_trajectory, vehicle_info.speed.data, self.vehicle_size)
        return vehicle_info

    def update(self, time: int, x: float, y: float, direction: int, speed: float) -> List[Point]:
        """
        function to generate the future trajectory of the vehicle

        (note: right now, it's constant velocity  assumption. to see how to incorporate frenet
        system in it, check the predictions code in Frenet directory)

        args:
            time: time horizon over which we want to predict the future trajectory
            x, y: position of the vehicle
            direction: current heading angle of the vehicle
            speed: current velocity of the vehicle 
        """

        f_x, f_y = [], []
        if direction == 0:  # up
            for t in range(time):
                x = x + np.random.randn() * t * 0.005
                y = y + speed * t * 0.005
                f_x.append(x)
                f_y.append(y)
        elif direction == 1:  # down
            for t in range(time):
                x = x + np.random.randn() * t * 0.005
                y = y - speed * t * 0.005
                f_x.append(x)
                f_y.append(y)
        elif direction == 2:  # right
            for t in range(time):
                x = x + speed * t * 0.005
                y = y + np.random.randn() * t * 0.005
                f_x.append(x)
                f_y.append(y)
        else:  # left
            for t in range(time):
                x = x - speed * t * 0.005
                y = y + np.random.randn() * t * 0.005
                f_x.append(x)
                f_y.append(y)

        points = self.ls_to_point(f_x, f_y)
        return points

    def add_noise(self, time_steps: int, trajectory: List[List[float]], velocity: float, radius: float, growth_rate: float = 1.0) -> List:
        """
        function to add circular noise to the future predicted position 

        returns a list of circular noise and time step: [noise(trajectory), time]

        args:
            time_steps: horizon length
            trajectory: future trajectory estimated 
            velocity: current velocity of the vehicle
            radius: noise size, initialized roughly by the size of the vehicle
            growth_rate: growth rate of the noise due to increase in uncertainity in the future
        """

        noise = []

        for time in range(time_steps):
            if growth_rate < 1.4:
                growth_rate += velocity * (time/50)

            size = growth_rate * radius

            noise.append([trajectory[time], size])

        return noise

    def intersectionArea(self, X1: float, Y1: float, R1: float, X2: float, Y2: float, R2: float) -> float:
        """
        calculate the overlap area of two circles

        args: 
            X1, Y1, R1: (X1, Y1) represents the center of circle1 with radius R1
            X2, Y2, R2: (X2, Y2) represents the center of circle2 with radius R2
        """

        math.pi = 3.14

        d = np.sqrt(((X2 - X1) * (X2 - X1)) + ((Y2 - Y1) * (Y2 - Y1)))

        if (d > R1 + R2):
            ans = 0

        elif (d <= (R1 - R2) and R1 >= R2):
            ans = math.floor(math.pi * R2 * R2)

        elif (d <= (R2 - R1) and R2 >= R1):
            ans = math.floor(math.pi * R1 * R1)

        else:
            alpha = math.acos(
                ((R1 * R1) + (d * d) - (R2 * R2)) / (2 * R1 * d)) * 2
            beta = math.acos(
                ((R2 * R2) + (d * d) - (R1 * R1)) / (2 * R2 * d)) * 2

            a1 = (0.5 * beta * R2 * R2) - (0.5 * R2 * R2 * math.sin(beta))
            a2 = (0.5 * alpha * R1 * R1) - (0.5 * R1 * R1 * math.sin(alpha))
            ans = math.floor(a1 + a2)

        return ans

    def overlap(self, first_vehicle: vehicle, first_vehicle_future_trajectory: list, second_vehicle: vehicle, second_vehicle_future_trajectory: list) -> float:
        """
        function to compute the normalized overlap area of two interescting circles
        at every prediction time step

        args:
            vehicle_1_data: future trajectory information with uncertainity size for vehicle 1, 
            vehicle_2_data: future trajectory information with uncertainity size for vehicle 2
        """

        vehicle_1_centers = first_vehicle_future_trajectory
        vehicle_1_size = self.vehicle_size

        vehicle_2_centers = second_vehicle_future_trajectory
        vehicle_2_size = self.vehicle_size

        overlap = self.intersectionArea(vehicle_1_centers.x, vehicle_1_centers.y, vehicle_1_size,
                                        vehicle_2_centers.x, vehicle_2_centers.y, vehicle_2_size)

        normalized_overlap = overlap / \
            (math.pi * (vehicle_1_size**2 + vehicle_2_size**2))

        return normalized_overlap

    def collision(self, first_vehicle: vehicle, second_vehicle: vehicle):
        """
        function to check collision at each time step

        args:
            first_vehicle: information about the first vehicle
            second_vehicle: information about the second vehicle
        """

        for t in range(self.time_steps):
            overlap_area = self.overlap(
                first_vehicle, first_vehicle.future_trajectory[t], second_vehicle, second_vehicle.future_trajectory[t])
            if overlap_area > self.max_collision_allowance:
                return True

    def collision_alert(self, env: queue):
        """
        function to alert the vehicle of the collision

        (need to pass on this information to the communication system)

        args:
            env: environment information received
        """

        for first_vehicle, second_vehicle in itertools.combinations(env.info, 2):
            if self.collision(first_vehicle, second_vehicle):
                print("!! ALERT : collision between", first_vehicle.id,
                      "and", second_vehicle.id , "!!")

    def point_on_lane(self, start: List[float], end: List[float], point: List[float]) -> List[float]:
        """
        function to get the nearest point on the lane from the vehicle

        args:
            start: (x, y) coordinate of the starting point of the lane
            end: (x, y) coordinate of the ending point of the lane
            point: (x, y) coordinate of the vehicle
        """

        dx, dy = end[0] - start[0], end[1] - start[1]
        det = dx*dx + dy*dy
        a = (dy*(point[1]-start[1]) + dx*(point[0]-start[0])) / det
        return start[0] + a*dx, start[1] + a*dy

    def closest_index(self, lane: List[List[float]], x: int, y: int) -> int:
        """
        function to get the index of the lane list

        (use in frenet system)

        args:
            lane: lane center information
        """

        return bisect.bisect_left(lane, (x, y))

    def ls_to_point(self, ls_x: List[float], ls_y: List[float]) -> List[Point]:
        """
        function to convert a 2D point in a geometry msgs Point type

        args:
            ls_x, ls_y: x, y list of points
        """

        points = []
        for i in range(len(ls_x)):
            points.append(Point(ls_x[i], ls_y[i], 0))
        return points

    def mark(self, id, direction, trajectory):
        """
        function to mark the positions on rviz

        args:
            id: id of the vehicle
            direction: heading direction of the vehicle (explained in README)
            trajectory: future trajectory of the vehicle
        """

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "vehicle"
        marker.type = marker.POINTS
        marker.action = 0
        marker.id = id.data
        marker.points = trajectory
        marker.pose.orientation.w = direction.data
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0

        if direction.data == 1:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.nsecs = 75000000
        markerPub.publish(marker)
        rate.sleep()


class Lanes:
    def __init__(self,) -> None:
        """
        class for accessing the lane information

        params:
            mid_lanes: start and end points of the mid lanes. as of now, they are hard-coded because we have not done camera calibration
        """

        self.mid_lanes = np.flip(np.array([[418.17932825, -458.47816553, 428.60006397, -168.4146595],
                                           [468.05591932, -439.93474339,
                                               496.06556125, -189.02849181],
                                           [730.6705301, 75.1316708,
                                               215.12283138, 73.98304454],
                                           [730.85281032, 254.21177936, 84.52455505, 257.48031548]]))
        # self.markLanes()

    def get_lane(self, vehicle: vehicle) -> List[float]:
        """
        function to get the closest lane from the vehicle

        (used in Frenet system, check in the frenet directory)

        args:
            vehicle: complete information of the vehicle
        """

        for line in self.mid_lanes:
            p1 = np.array([line[0], line[1]])
            p2 = np.array([line[2], line[3]])
            p3 = np.array([vehicle.x_position,
                           vehicle.y_position])
            dist = abs(np.cross(p2-p1, p3-p1)/np.linalg.norm(p2-p1))
            if dist < d:
                start_point, end_point = Point(), Point()
                start_point.x, start_point.y = line[0], line[1]
                end_point.x, end_point.y = line[2], line[3]
                d = dist
        return start_point, end_point

    def build_lane(self, start: Point, end: Point, num_points=100) -> List[List[float]]:
        """
        function to get the lane from the start and end point

        args:
            start: start point of the lane
            end: end point of the lane
            num_points: discretization of the lane
        """

        x_ = np.linspace(start.x, end.x, num_points)
        y_ = np.linspace(start.y, end.y, num_points)
        # return list(zip(x_, y_))    # update later
        return x_, y_

    def distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """
        function to get the euclidean distance between the points

        args:
            x1, y1: point 1
            x2, y2: point 2
        """

        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    def markLanes(self, vehicle_info: vehicle):
        """
        function to mark the lanes on rviz to which vehicle is closest to

        args:
            vehicle_info: complete vehicle state
        """

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "lanes"
        marker.type = marker.POINTS
        marker.action = 0
        lane_info = Lanes()
        lanes_x, lanes_y = lane_info.build_lane(
            vehicle_info.start_point, vehicle_info.end_point)
        lanes = self.ls_to_point(lanes_x, lanes_y)
        marker.points = lanes
        marker.scale.x = 10.0
        marker.scale.y = 10.0
        marker.scale.z = 10.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.nsecs = 75000000
        markerPub.publish(marker)
        rate.sleep()

    def get_s_map(self, lane: List) -> List:
        """
        function to get the s-map for frenet system (check usage in frenet folder)

        args:
            lane: points of the lane to which vehicle is closest to
        """

        s_map = np.array([], dtype=np.float64)
        accumulated_distance = 0.0
        prev_point = None
        for point in lane:
            if prev_point != None:
                accumulated_distance += self.distance(
                    prev_point[0], prev_point[1], point[0], point[1])
            s_map = np.append(s_map, accumulated_distance)
            prev_point = point

        return s_map


if __name__ == '__main__':
    # publisher node for predictions
    rospy.init_node('predictions', anonymous=True)

    # publish the update environment information
    env_pub = rospy.Publisher('predict', queue, queue_size=10)

    # marker publisher for vehicle visualization in rviz
    markerPub = rospy.Publisher('vehicle', Marker, queue_size=10)

    # marker publisher for closest lane visualization in rviz
    centerLaneMarker = rospy.Publisher('lanes', Marker, queue_size=10)

    # publishing rate
    rate = rospy.Rate(10000)

    # call the class for collision detection
    Predictions()

    try:
        # keep publishing till interrupt
        rospy.spin()

    except KeyboardInterrupt:
        print("exit")
