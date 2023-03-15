#! /usr/bin/env python3

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, Vector3, Pose, PoseWithCovariance, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

# main function
if __name__ == '__main__':
    rospy.init_node('init_odom_pose')
    pub = rospy.Publisher('/bot_controller/odom', Odometry, queue_size=10)

    rate = rospy.Rate(2)

    odom = Odometry()
    pos_car_1 = Point(2.0, 0.0, 1.0)
    yaw_car_1 = 0
    v_1 = 1
    lin_vel_1 = Vector3(v_1, 0.0, 0.0)
    ang_vel_1 = Vector3(0.0, 0.0, 0.0)
    q_1 = quaternion_from_euler(0, 0, yaw_car_1)
    car_1_pose = Pose(pos_car_1, Quaternion(q_1[0], q_1[1], q_1[2], q_1[3]))
    car_1_twist = Twist(lin_vel_1, ang_vel_1)
    covariance_1 = [[0 for _ in range(6)] for _ in range(6)]
    car_1_pose_with_covariance = PoseWithCovariance(car_1_pose, covariance_1)
    car_1_odom = Odometry(Header, "base_footprint", car_1_pose_with_covariance, car_1_twist) 
    while not rospy.is_shutdown():
        pub.publish(car_1_odom)
        rate.sleep()