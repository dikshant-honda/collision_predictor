from py_msgs.std_msgs import *
import numpy as np

'''
TO DO:
1. just cross-check all the messages are written correctly
2. covariance matrix thing, better way to design it
'''

class Vector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Accel:
    def __init__(self, linear, angular):
        self.linear = Vector3(linear.x, linear.y, linear.z)
        self.angular = Vector3(angular.x, angular.y, angular.w)

class Point:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class Quaternion:
    def __init__(self, x=0, y=0, z=0, w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class Pose:
    def __init__(self, pose, quaternion):
        self.position = Point(pose.x , pose.y, pose.z)
        self.orientation = Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

class Pose2D:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class PoseStamped:
    def __init__(self, header, pose):
        self.header = Header(header.seq, header.stamp, header.frame_id)
        self.pose = Pose(pose.position, pose.orientation)

class Twist:
    def __init__(self, linear, angular):
        self.linear = Vector3(linear.x, linear.y, linear.z)
        self.angular = Vector3(angular.x, angular.y, angular.z)

class TwistStamped:
    def __init__(self, header, twist):
        self.header = Header(header.seq, header.stamp, header.frame_id)
        self.twist = Twist(twist.linear, twist.angular)

class PoseWithCovariance:
    def __init__(self, pose, covariance):
        self.pose = Pose(pose.position, pose.orientation)
        pos_deviation = 0.5
        ori_deviation = np.pi/6
        covariance = np.zeros((6,6))
        covariance[0] = pos_deviation**2
        covariance[6+1] = 0.001
        covariance[12+2] = 0.0001
        covariance[18+3] = ori_deviation**2
        covariance[24+4] = ori_deviation**3
        covariance[30+5] = ori_deviation**4
        self.covariance = covariance     

class TwistWithCovariance:
    def __init__(self, twist, covariance):
        self.twist = Twist(twist.linear, twist.angular)
        pos_deviation = 0.5
        ori_deviation = np.pi/6
        covariance = np.zeros((6,6))
        covariance[0] = pos_deviation**2
        covariance[6+1] = 0.001
        covariance[12+2] = 0.0001
        covariance[18+3] = ori_deviation**2
        covariance[24+4] = ori_deviation**3
        covariance[30+5] = ori_deviation**4
        self.covariance = covariance 

class AccelWithCovariance:
    def __init__(self, accel, covariance):
        self.accel = Accel(accel.linear, accel.angular)
        pos_deviation = 0.5
        ori_deviation = np.pi/6
        covariance = np.zeros((6,6))
        covariance[0] = pos_deviation**2
        covariance[6+1] = 0.001
        covariance[12+2] = 0.0001
        covariance[18+3] = ori_deviation**2
        covariance[24+4] = ori_deviation**3
        covariance[30+5] = ori_deviation**4
        self.covariance = covariance 

# TEST
# pt = Point()
# quat = Quaternion()
# pose = Pose(pt,quat)
# head = std_msgs.Header()
# poses = PoseStamped(head,pose)
# print(poses.pose.orientation.w)