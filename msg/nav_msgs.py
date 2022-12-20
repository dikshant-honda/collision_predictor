import std_msgs
import geometry_msgs

class Path:
    def __init__(self, header, posestamped):
        self.header = header
        self.poses = posestamped

# TEST
# pt = geometry_msgs.Point()
# quat = geometry_msgs.Quaternion()
# head = std_msgs.Header()
# pose = geometry_msgs.Pose(pt,quat)
# poses = geometry_msgs.PoseStamped(head, pose)
# print(poses.pose.orientation.x)