#! /usr/bin/env python3

import rospy
import numpy as np

from geometry_msgs.msg import Point
from collision_predictor.msg import vehicle, environment


class Predictions:
    def __init__(self) -> None:
        self.predictions_pub = rospy.Publisher('prediction_info', environment, queue_size=10)
        rospy.init_node('future trajectory predictions')
        self.rate = rospy.Rate(10000)
        self.sub = rospy.Subscriber('env_info', environment, self.callback)

    def callback(self, data):
        self.env = data
        self.get_vehicle_data(self.env)

    def get_vehicle_data(self, env: environment):
        for vehicle_info in env.vehicle_states:
            vehicle_info = self.get_future_trajectory(vehicle_info)
        self.predictions_pub.publish(env)
        self.rate.sleep()

    def get_future_trajectory(self, vehicle_info: vehicle):
        '''
        update this function using various approaches:
        1. predicting the trajectory from the past data
        2. constant velocity assumption
        3. using frenet function to make use of the lane information
        4. using intelligent driver model (IDM)
        '''
        vehicle_info.future_trajectory.append(Point(vehicle_info.x_position, vehicle_info.y_position, 0))
        return vehicle_info

if __name__ == '__main__':
    # subscriber node for detection
    rospy.init_node('update environment', anonymous=True)
    Predictions()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("exit")
