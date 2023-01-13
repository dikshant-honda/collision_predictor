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
    pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=10)
    pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=10)
    pub3 = rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=10)
    pub4 = rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=10)
    pub5 = rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(2)

    # speed of the turtlebots
    lin_vel_1 = 0.2
    ang_vel_1 = 0.0
    lin_vel_2 = 0.1
    ang_vel_2 = 0.0
    lin_vel_3 = 0.1
    ang_vel_3 = 0.0
    lin_vel_4 = 0.1
    ang_vel_4 = 0.0
    lin_vel_5 = 0.1
    ang_vel_5 = 0.0

    while not rospy.is_shutdown():
        update_pos(pub1, lin_vel_1, ang_vel_1)
        update_pos(pub2, lin_vel_2, ang_vel_2)
        update_pos(pub3, lin_vel_3, ang_vel_3)
        update_pos(pub4, lin_vel_4, ang_vel_4)
        update_pos(pub5, lin_vel_5, ang_vel_5)
        rate.sleep()