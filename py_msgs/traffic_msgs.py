import std_msgs
import nav_msgs
import geometry_msgs

# traffic msgs dataclasses for python based simulation usage. not required anymore!

class Point2D:
    def __init__(self,x_init,y_init):
        self.x = x_init
        self.y = y_init

class Point3D:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

class PathWithSpeed:
    def __init__(self, path, curvature_speeds):
        self.path = nav_msgs.Path(path.header, path.posestamped)  
        curvature_speed_arr = []
        for i in range(len(curvature_speeds)):
            curvature_speed_arr.append(i)               
        self.curvature_speeds = curvature_speed_arr   

class CenterLanes:
    def __init__(self, header, center_lines, ids, lanes_width, speed_limits, acc_min_limits, acc_max_limits, jrk_min_limits, jrk_max_limits, crossing_ad_points, merge_start_points, merge_end_points, stop_bar_points):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        center_line_arr = []
        for i in range(center_lines):
            center_line_arr.append(i.path, i.curvature_speeds)
        id_arr = []
        for i in range(len(ids)):
            id_arr.append(i)
        lanes_width_arr = []
        for i in range(len(lanes_width)):
            lanes_width_arr.append(i)
        speed_limits_arr = []
        for i in range(len(speed_limits)):
            speed_limits_arr.append(i)
        acc_min_limits_arr = []
        for i in range(len(acc_min_limits)):
            acc_min_limits_arr.append(i)
        acc_max_limits_arr = []
        for i in range(len(acc_max_limits)):
            acc_max_limits_arr.append(i)
        jrk_min_limits_arr = []
        for i in range(len(jrk_min_limits)):
            jrk_min_limits_arr.append(i)
        jrk_max_limits_arr = []
        for i in range(len(jrk_max_limits)):
            jrk_max_limits_arr.append(i)
        crossing_ad_points_arr = []
        for i in range(len(crossing_ad_points)):
            crossing_ad_points_arr.append(i)
        merge_start_points_arr = []
        for i in range(merge_start_points):
            merge_start_points_arr.append(i)
        merge_end_points_arr = []
        for i in range(merge_end_points):
            merge_end_points_arr.append(i)
        stop_bar_points_arr = []
        for i in range(len(stop_bar_points)):
            stop_bar_points_arr.append(i)
        
        self.center_lines = center_line_arr
        self.ids = id_arr
        self.lanes_width = lanes_width_arr
        self.speed_limits = speed_limits_arr
        self.acc_min_limits = acc_min_limits_arr
        self.acc_max_limits = acc_max_limits_arr
        self.jrk_min_limits = jrk_min_limits_arr
        self.jrk_max_limits = jrk_max_limits_arr
        self.crossing_ad_points = crossing_ad_points_arr
        self.merge_start_points = merge_start_points_arr
        self.merge_end_points = merge_end_points_arr
        self.stop_bar_points = stop_bar_points_arr       
    
class Prediction:
    def __init__(self, lane_id, trajectories, agent_id, width, length, dt):
        self.lane_id = lane_id
        self.trajectories = IntentionTrajectory(trajectories.trajectory_probability, trajectories.trajectory_estimated, trajectories.trajectory_uncertainity)
        self.agent_id = agent_id
        self.width = width
        self.length = length
        self.dt = dt

class PredictionArray:
    def __init__(self, header, predictions):
        self.Header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        prediction_arr = []
        for i in range(len(predictions)):
            prediction_arr.append(i)
        self.predictions = prediction_arr

class PredictionLanes:
    def __init__(self, header, ids, vehicles_prediction):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        id_arr = []
        for i in range(len(ids)):
            id_arr.append(i)
        self.ids = id_arr
        self.vehicles_prediction = PredictionArray(header, vehicles_prediction)

class Waypoint:
    def __init__(self, pose, twist):
        header = std_msgs.Header(seq=None, stamp=None, frame_id=None)
         # header info not given so doing without it, change later maybe
        self.pose = geometry_msgs.PoseStamped(header, pose)
        self.twist = geometry_msgs.TwistStamped(header, twist)

class WaypointArray:
    def __init__(self, header, waypoints):
        self.Header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        waypoint_arr = []
        for i in range(len(waypoints)):
            waypoint_arr.append(i)
        self.waypoints = waypoint_arr

class IntentionTrajectory:
    def __init__(self, trajectory_probability, trajectory_estimated, trajectory_uncertainity):
        self.trajectory_probability = trajectory_probability
        header = std_msgs.Header(seq=None, stamp=None, frame_id=None)
         # header info not given so doing without it, change later maybe
        self.trajectory_estimated = WaypointArray(header, trajectory_estimated)
        self.trajectory_uncertainity = WaypointArray(header, trajectory_uncertainity)

class PerceptionLanes:
    def __init__(self, header, ids, vehicles):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        id_arr = []
        for i in range(len(ids)):
            id_arr.append(i)
        self.ids = id_arr
        self.vehicles = VehicleStateArray(header, vehicles)

class VehicleStateArray:
    def __init__(self, header, vehicles):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        vehicle_arr = []
        for i in range(len(vehicles)):
            vehicle_arr.append(VehicleState(i.header, i.lifetime_id, i.local_id, i.width, i.length, i.s, i.d, i.pose, i.twist, i.accel))
        self.vehicles = vehicle_arr

class VehicleState:
    def __init__(self, header, lifetime_id, local_id, width, length, s, d, pose, twist, accel):
        self.header = std_msgs.Header(header.seq, header.stamp, header.frame_id)
        self.lifetime_id = lifetime_id
        self.local_id = local_id
        self.width = width
        self.length = length
        self.s = s
        self.d = d
        # change the covariance=None part in ros implementation
        self.pose = geometry_msgs.PoseWithCovariance(pose.pose, covariance=None)
        self.twist = geometry_msgs.TwistWithCovariance(twist.twist, covariance=None)
        self.accel = geometry_msgs.AccelWithCovariance(accel.accel, covariance=None)

# TEST
# pt = geometry_msgs.Point()
# quat = geometry_msgs.Quaternion()
# lin = geometry_msgs.Vector3()
# ang = geometry_msgs.Vector3()
# head = std_msgs.Header()
# pose = geometry_msgs.Pose(pt,quat)
# poses = geometry_msgs.PoseStamped(head, pose)
# twist = geometry_msgs.Twist(lin, ang)
# twists = geometry_msgs.TwistStamped(head, twist)
# waypoint = Waypoint(pose, twist)
# print(waypoint.pose.pose.orientation.w)