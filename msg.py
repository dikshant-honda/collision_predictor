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
        self.header = header
        self.predictions = self.Prediction()

    def PredictionLanes(self, header, ids):
        self.header = header
        self.ids = ids                              # int32[]
        self.vehicles_prediction = self.PredictionArray()

    def Waypoint(self, pose, twist):
        self.pose = pose                            # geometry_msgs/PoseStamped
        self.twist = twist                          # geomtery_msgs/TwistStamped

    def WaypointArray(self, header):
        self.header = header
        self.waypoints = self.Waypoint()

    def IntentionTrajectory(self, trajectory_probability):
        self.trajectory_probability = trajectory_probability
        self.trajectory_estimated = self.WaypointArray()
        self.trajectory_uncertainity = self.WaypointArray()

    def PerceptionLanes(self, header, ids):
        self.header = header
        self.ids = ids
        self.vehicles = self.VehicleStateArray()

    def VehicleStateArray(self, header):
        self.header = header
        self.vehicles = self.VehicleState()

    def VehicleState(self, header, lifetime_id, local_id, width, length, s, d, pose, twist, accel):
        self.header = header
        self.lifetime_id = lifetime_id
        self.local_id = local_id
        self.width = width
        self.length = length
        self.s = s
        self.d = d
        self.pose = pose                            # geometry_msgs/PoseWithCovariance
        self.twist = twist                          # geometry_msgs/TwistWithCovariance
        self.accel = accel                          # geometry_msgs/AccelWithCovariance