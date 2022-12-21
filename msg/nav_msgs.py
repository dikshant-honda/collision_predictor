import std_msgs
import geometry_msgs

class Path:
    def __init__(self, header, posestamped):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        self.poses = geometry_msgs.PoseStamped(posestamped.header, posestamped.pose)

# TEST
# pt = geometry_msgs.Point()
# quat = geometry_msgs.Quaternion()
# head = std_msgs.Header()
# pose = geometry_msgs.Pose(pt,quat)
# poses = geometry_msgs.PoseStamped(head, pose)
# print(poses.pose.orientation.x)