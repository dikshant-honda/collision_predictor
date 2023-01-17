from py_msgs.std_msgs import *
from py_msgs.geometry_msgs import *

class Path:
    def __init__(self, header, poses):
        self.header = Header(header.seq, header.stamp, header.frame_id)
        pose_arr = []
        for i in range(len(poses)):
            pose_arr.append(PoseStamped(self.header, poses[i].pose))
        self.poses = pose_arr

class Odometry:
    def __init__(self, header, pose, twist):
        self.header = Header(header.seq, header.stamp, header.frame_id)
        self.pose = PoseWithCovariance(pose.pose, pose.covariance)
        self.twist = TwistWithCovariance(twist.twist, twist.covariance)

# TEST
# pt = geometry_msgs.Point()
# quat = geometry_msgs.Quaternion()
# head = std_msgs.Header()
# pose = geometry_msgs.Pose(pt,quat)
# poses = geometry_msgs.PoseStamped(head, pose)
# print(poses.pose.orientation.x)