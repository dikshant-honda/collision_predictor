import std_msgs

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

class PoseStamped:
    def __init__(self, header, pose):
        self.header = header
        self.pose = Pose(pose.position, pose.orientation)

class Twist:
    def __init__(self, linear, angular):
        self.linear = Vector3(linear.x, linear.y, linear.z)
        self.angular = Vector3(angular.x, angular.y, angular.z)

class TwistStamped:
    def __init__(self, header, twist):
        self.header = header
        self.twist = Twist(twist.linear, twist.angular)

def PoseWithCovariance(self):
    self.pose = self.Pose()
    # self.covariance = covariance     # of size 36

def TwistWithCovariance(self):
    self.twist = self.Twist()
    # self.covariance = covariance

def AccelWithCovariance(self):
    self.accel = self.Accel()
    # self.covariance = covariance

# TEST
# pt = Point()
# quat = Quaternion()
# pose = Pose(pt,quat)
# head = std_msgs.Header()
# poses = PoseStamped(head,pose)