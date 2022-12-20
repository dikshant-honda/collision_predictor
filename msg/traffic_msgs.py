import std_msgs
import nav_msgs
import geometry_msgs

class Point2D:
    def __init__(self,x_init,y_init):
        self.x = x_init
        self.y = y_init

class Point3D():
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

def PathWithSpeed(self, path, curvature_speeds):
    self.path = nav_msgs.Path()                 
    self.curvature_speeds = curvature_speeds    

def CenterLanes(self):
    self.lines = self.PathWithSpeed()
    
def Prediction(self, lane_id, agent_id, width, length, dt):
    self.lane_id = lane_id
    self.trajectories = self.IntentionTrajectory()
    self.agent_id = agent_id
    self.width = width
    self.length = length
    self.dt = dt

def PredictionArray(self, Header):
    self.Header = std_msgs.Header()
    self.predictions = self.Prediction()

def PredictionLanes(self, Header, ids):
    self.Header = std_msgs.Header()
    self.ids = ids                              # int32[]
    self.vehicles_prediction = self.PredictionArray()

class Waypoint:
    def __init__(self, pose, twist):
        geometry_msgs.__init__(self, pose)
        self.pose = geometry_msgs.Pose(pose)                            # geometry_msgs/PoseStamped
        self.twist = geometry_msgs.Twist(twist)                          # geomtery_msgs/TwistStamped

def WaypointArray(self, Header):
    self.Header = std_msgs.Header()
    self.waypoints = self.Waypoint()

def IntentionTrajectory(self, trajectory_probability):
    self.trajectory_probability = trajectory_probability
    self.trajectory_estimated = self.WaypointArray()
    self.trajectory_uncertainity = self.WaypointArray()

def PerceptionLanes(self, Header, ids):
    self.Header = std_msgs.Header()
    self.ids = ids
    self.vehicles = self.VehicleStateArray()

def VehicleStateArray(self, Header):
    self.Header = std_msgs.Header()
    self.vehicles = self.VehicleState()

def VehicleState(self, Header, lifetime_id, local_id, width, length, s, d, pose, twist, accel):
    self.Header = std_msgs.Header()
    self.lifetime_id = lifetime_id
    self.local_id = local_id
    self.width = width
    self.length = length
    self.s = s
    self.d = d
    self.pose = pose                            # geometry_msgs/PoseWithCovariance
    self.twist = twist                          # geometry_msgs/TwistWithCovariance
    self.accel = accel                          # geometry_msgs/AccelWithCovariance


