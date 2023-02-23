#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def update_pos(pub, lin_vel, ang_vel):
    move = Twist()
    move.linear.x = lin_vel
    move.angular.z = ang_vel
    pub.publish(move)


# main function
if __name__ == '__main__':
    rospy.init_node('move_robot_node')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(2)

    # speed of the turtlebots
    lin_vel = 3
    ang_vel = 0.0

    while not rospy.is_shutdown():
        update_pos(pub, lin_vel, ang_vel)
        rate.sleep()