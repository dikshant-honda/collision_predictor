#! /usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Point
from collision_predictor.msg import vehicle, environment


class Predictions:
    def __init__(self) -> None:
        self.sub = rospy.Subscriber('env', environment, self.callback)

    def callback(self, data):
        self.env = data
        self.get_vehicle_data(self.env)

    def get_vehicle_data(self, env: environment):
        for vehicle_info in env.vehicle_states:
            vehicle_info = self.get_future_trajectory(vehicle_info)
        env_pub.publish(env)

    def get_future_trajectory(self, vehicle_info: vehicle):
        '''
        update this function using various approaches:
        1. predicting the trajectory from the past data
        2. constant velocity assumption
        3. using frenet function to make use of the lane information
        4. using intelligent driver model (IDM)
        '''
        vehicle_info.future_trajectory = self.update(
            vehicle_info.x_position.data, vehicle_info.y_position.data, vehicle_info.direction.data, vehicle_info.speed.data)
        return vehicle_info

    def update(self, x, y, direction, speed):
        f_x, f_y = [], []
        for t in range(10):
            x = x + direction * speed * t * 0.005
            y = y + direction * np.random.randn() * t * 0.005
            f_x.append(x)
            f_y.append(y)
        points = self.ls_to_point(f_x, f_y)
        return points

    def ls_to_point(self, ls_x, ls_y):
        points = []
        for i in range(len(ls_x)):
            points.append(Point(ls_x[i], ls_y[i], 0))
        return points


if __name__ == '__main__':
    # publisher node for predictions
    rospy.init_node('predictions', anonymous=True)
    env_pub = rospy.Publisher('predict', environment, queue_size=10)

    Predictions()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("exit")
