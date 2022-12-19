class std_msgs:
    def header(self, header):
        self.header = header

class nav_msgs:
    def Path(self):
        self.header = std_msgs.header()
        self.poses = geometry_msgs.PoseStamped()

class geometry_msgs:
    def Vector3(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def Twist(self):
        self.linear = self.Vector3()
        self.angular = self.Vector3()
    
    def Accel(self):
        self.linear = self.Vector3()
        self.angular = self.Vector3()

    def Point(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def Quaternion(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def Pose(self):
        self.position = self.Point()
        self.orientation = self.Quaternion()

    def PoseStamped(self):
        self.header = std_msgs.header()
        self.pose = self.Pose()
        
    def TwistStamped(self):
        self.header = std_msgs.header()
        self.twist = self.Twist()

    def PoseWithCovariance(self):
        self.pose = self.Pose()
        # self.covariance = covariance     # of size 36

    def TwistWithCovariance(self):
        self.twist = self.Twist()
        # self.covariance = covariance
    
    def AccelWithCovariance(self):
        self.accel = self.Accel()
        # self.covariance = covariance
    

class traffic_msgs:
    def PathWithSpeed(self, path, curvature_speeds):
        self.path = path                            # nav_msgs/Path
        self.curvature_speeds = curvature_speeds    # float64[]

    def CenterLanes(self):
        self.lines = self.PathWithSpeed()
        
    def Prediction(self, lane_id, agent_id, width, length, dt):
        self.lane_id = lane_id
        self.trajectories = self.IntentionTrajectory()
        self.agent_id = agent_id
        self.width = width
        self.length = length
        self.dt = dt

    def PredictionArray(self, header):
        self.header = std_msgs.header()
        self.predictions = self.Prediction()

    def PredictionLanes(self, header, ids):
        self.header = std_msgs.header()
        self.ids = ids                              # int32[]
        self.vehicles_prediction = self.PredictionArray()

    def Waypoint(self, pose, twist):
        self.pose = pose                            # geometry_msgs/PoseStamped
        self.twist = twist                          # geomtery_msgs/TwistStamped

    def WaypointArray(self, header):
        self.header = std_msgs.header()
        self.waypoints = self.Waypoint()

    def IntentionTrajectory(self, trajectory_probability):
        self.trajectory_probability = trajectory_probability
        self.trajectory_estimated = self.WaypointArray()
        self.trajectory_uncertainity = self.WaypointArray()

    def PerceptionLanes(self, header, ids):
        self.header = std_msgs.header()
        self.ids = ids
        self.vehicles = self.VehicleStateArray()

    def VehicleStateArray(self, header):
        self.header = std_msgs.header()
        self.vehicles = self.VehicleState()

    def VehicleState(self, header, lifetime_id, local_id, width, length, s, d, pose, twist, accel):
        self.header = std_msgs.header()
        self.lifetime_id = lifetime_id
        self.local_id = local_id
        self.width = width
        self.length = length
        self.s = s
        self.d = d
        self.pose = pose                            # geometry_msgs/PoseWithCovariance
        self.twist = twist                          # geometry_msgs/TwistWithCovariance
        self.accel = accel                          # geometry_msgs/AccelWithCovariance