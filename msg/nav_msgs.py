import std_msgs
import geometry_msgs

class Path:
    def __init__(self, header, posestamped):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        pose_arr = []
        for i in range(len(posestamped)):
            pose_arr.append(geometry_msgs.PoseStamped(posestamped.header, posestamped.pose))
        self.poses = pose_arr

# TEST
# pt = geometry_msgs.Point()
# quat = geometry_msgs.Quaternion()
# head = std_msgs.Header()
# pose = geometry_msgs.Pose(pt,quat)
# poses = geometry_msgs.PoseStamped(head, pose)
# print(poses.pose.orientation.x)