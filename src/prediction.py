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
        self.sub = rospy.Subscriber('env_info', queue, self.callback)
        self.lanes = Lanes()
        self.time_steps = 50
        self.max_collision_allowance = 0.1

    def callback(self, data):
        self.env = data
        self.get_vehicle_data(self.env)

    def get_vehicle_data(self, env: queue) -> queue:
        for vehicle_info in env.info:
            vehicle_info = self.get_future_trajectory(vehicle_info)
        env_pub.publish(env)

    def get_future_trajectory(self, vehicle_info: vehicle) -> vehicle:
        vehicle_info.future_trajectory = self.update(self.time_steps,
                                                     vehicle_info.x_position.data, vehicle_info.y_position.data, vehicle_info.direction.data, vehicle_info.speed.data)
        self.mark(vehicle_info.id, vehicle_info.direction,
                  vehicle_info.future_trajectory)
        self.true_predictions = self.add_noise(
            self.time_steps, vehicle_info.future_trajectory, vehicle_info.speed.data, 5)
        return vehicle_info

    def collision(self, first_vehicle, second_vehicle):
        for t in range(self.time_steps):
            overlap_area = self.overlap(first_vehicle[t], second_vehicle[t])
            if overlap_area > self.max_collision_allowance:
                return True

    def intersectionArea(self, X1: float, Y1: float, R1: float, X2: float, Y2: float, R2: float) -> float:
        """
        calculate the overlap area of two circles

        args: 
            X1, Y1, R1: (X1, Y1) represents the center of circle1 with radius R1
            X2, Y2, R2: (X2, Y2) represents the center of circle2 with radius R2
        """

        np.pi = 3.14

        d = np.sqrt(((X2 - X1) * (X2 - X1)) + ((Y2 - Y1) * (Y2 - Y1)))

        if (d > R1 + R2):
            ans = 0

        elif (d <= (R1 - R2) and R1 >= R2):
            ans = math.floor(np.pi * R2 * R2)

        elif (d <= (R2 - R1) and R2 >= R1):
            ans = math.floor(np.pi * R1 * R1)

        else:
            alpha = math.acos(
                ((R1 * R1) + (d * d) - (R2 * R2)) / (2 * R1 * d)) * 2
            beta = math.acos(
                ((R2 * R2) + (d * d) - (R1 * R1)) / (2 * R2 * d)) * 2

            a1 = (0.5 * beta * R2 * R2) - (0.5 * R2 * R2 * math.sin(beta))
            a2 = (0.5 * alpha * R1 * R1) - (0.5 * R1 * R1 * math.sin(alpha))
            ans = math.floor(a1 + a2)

        return ans

    def overlap(self, vehicle_1_data: list, vehicle_2_data: list) -> float:
        """
        Function to compute the normalized overlap area of two interescting circles
        at every prediction time step

        args:
            vehicle_1_data: future trajectory information with uncertainity size for vehicle 1, 
            vehicle_2_data: future trajectory information with uncertainity size for vehicle 2
        """

        vehicle_1_centers = vehicle_1_data[0]
        vehicle_1_size = vehicle_1_data[1]

        vehicle_2_centers = vehicle_2_data[0]
        vehicle_2_size = vehicle_2_data[1]

        overlap = self.intersectionArea(vehicle_1_centers.x, vehicle_1_centers.y, vehicle_1_size,
                                        vehicle_2_centers.x, vehicle_2_centers.y, vehicle_2_size)

        normalized_overlap = overlap / \
            (np.pi * (vehicle_1_size**2 + vehicle_2_size**2))

        return normalized_overlap

    def collision_alert(self, env: queue):
        for first_vehicle, second_vehicle in itertools.combinations(env.info, 2):
            if self.collision(first_vehicle, second_vehicle):
                print("collision between", first_vehicle.id,
                      "and", second_vehicle.id)

    def point_on_lane(self, start: List[float], end: List[float], point: List[float]) -> List[float]:
        dx, dy = end[0] - start[0], end[1] - start[1]
        det = dx*dx + dy*dy
        a = (dy*(point[1]-start[1]) + dx*(point[0]-start[0])) / det
        return start[0] + a*dx, start[1] + a*dy

    def closest_index(self, lane: List[List[float]], x: int, y: int) -> int:
        return bisect.bisect_left(lane, (x, y))

    def update(self, time: int, x: float, y: float, direction: int, speed: float) -> List[Point]:
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

    def ls_to_point(self, ls_x: List[float], ls_y: List[float]) -> List[Point]:
        points = []
        for i in range(len(ls_x)):
            points.append(Point(ls_x[i], ls_y[i], 0))
        return points

    def add_noise(self, time_steps: int, trajectory: List[List[float]], velocity: float, radius: float, growth_rate: float = 1.0) -> List:
        """
        function to aodd circular noise to the future predicted position 

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

    def mark(self, id, direction, trajectory):
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

        args:
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
        x_ = np.linspace(start.x, end.x, num_points)
        y_ = np.linspace(start.y, end.y, num_points)
        # return list(zip(x_, y_))    # update later
        return x_, y_

    def distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return np.sqrt((x1-x2)**2 + (y1-y2)**2)

    def markLanes(self, vehicle_info: vehicle):
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

    def get_s_map(self, lane):
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


class Frenet:
    def __init__(self, s=0, d=0):
        self.s = s
        self.d = d


if __name__ == '__main__':
    # publisher node for predictions
    rospy.init_node('predictions', anonymous=True)
    env_pub = rospy.Publisher('predict', queue, queue_size=10)
    markerPub = rospy.Publisher('vehicle', Marker, queue_size=10)
    centerLaneMarker = rospy.Publisher('lanes', Marker, queue_size=10)
    rate = rospy.Rate(10000)
    Predictions()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("exit")
