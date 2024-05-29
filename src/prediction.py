#! /usr/bin/env python3

import rospy
import numpy as np
import bisect

from geometry_msgs.msg import Point
from collision_predictor.msg import vehicle
from visualization_msgs.msg import Marker
from multi_vehicle_tracking.msg import queue


class Predictions:
    def __init__(self) -> None:
        self.sub = rospy.Subscriber('env_info', queue, self.callback)
        self.lanes = Lanes()

    def callback(self, data):
        self.env = data
        self.get_vehicle_data(self.env)

    def get_vehicle_data(self, env: queue):
        for vehicle_info in env.info:
            vehicle_info = self.get_future_trajectory(vehicle_info)
        env_pub.publish(env)

    def get_future_trajectory(self, vehicle_info: vehicle):
        '''
        update this function using various approaches:
        1. predicting the trajectory from the past data
        2. constant velocity assumption # done
        3. using frenet function to make use of the lane information
        '''
        vehicle_info.future_trajectory = self.update(
            vehicle_info.x_position.data, vehicle_info.y_position.data, vehicle_info.direction.data, vehicle_info.speed.data)
        self.mark(vehicle_info.id, vehicle_info.direction,
                  vehicle_info.future_trajectory)
        return vehicle_info

    def point_on_lane(self, start, end, point):
        dx, dy = end[0] - start[0], end[1] - start[1]
        det = dx*dx + dy*dy
        a = (dy*(point[1]-start[1]) + dx*(point[0]-start[0])) / det
        return start[0] + a*dx, start[1] + a*dy

    def closest_index(self, lane, x, y):
        return bisect.bisect_left(lane, (x, y))

    def update(self, x, y, direction, speed):
        f_x, f_y = [], []
        for t in range(30):
            x = x + direction * np.random.randn() * t * 0.005
            y = y + direction * speed * t * 0.005
            f_x.append(x)
            f_y.append(y)
        points = self.ls_to_point(f_x, f_y)
        return points

    def ls_to_point(self, ls_x, ls_y):
        points = []
        for i in range(len(ls_x)):
            points.append(Point(ls_x[i], ls_y[i], 0))
        return points

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

    def get_lane(self, vehicle):
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

    def build_lane(self, start: Point, end: Point, num_points=100):
        x_ = np.linspace(start.x, end.x, num_points)
        y_ = np.linspace(start.y, end.y, num_points)
        # return list(zip(x_, y_))    # update later
        return x_, y_

    def distance(self, x1, y1, x2, y2):
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
