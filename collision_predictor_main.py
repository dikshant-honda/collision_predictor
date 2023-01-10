import math
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs import *
from geometry_msgs import *
from tf import *
from frenet import *

'''
TO DO: added in the notebook
'''
class Vehicle:
    count = 0
    ids = []
    vehicle_dynamics = {}        # [pose, twist, s, d, past, future]
    def __init__(self, pose, twist, s, d, past_waypoints, future_waypoints, id):
        self.pose = geometry_msgs.Pose(pose.position, pose.orientation)
        self.twist = geometry_msgs.Twist(twist.linear, twist.angular)
        self.s = s 
        self.d = d
        self.past_waypoints = past_waypoints
        self.future_waypoints = future_waypoints
        self.id = id
        Vehicle.vehicle_dynamics[self.id] = [self.pose, self.twist, self.s, self.d, self.past_waypoints, self.future_waypoints]

    # add the vehicle with unique ids to vehicles array
    def add_vehicle(self, veh):
        if self.id not in Vehicle.ids:
            Vehicle.count += 1
            Vehicle.ids.append(self.id) 

    # register the ego vehicle
    def register_ego(self, ego):
        self.add_vehicle(ego)

    # register the vehicle into dynamics
    def register_vehicle(self, ego, veh):
        if ego_vicinity(ego, veh):
            # print("start registering")
            self.add_vehicle(veh)

# check for the vehicles which are in the vicinity of the ego vehicle
def ego_vicinity(ego, veh):
    ego_pos = ego.pose.position
    veh_pos = veh.pose.position
    if distance(ego_pos.x, ego_pos.y, veh_pos.x, veh_pos.y) < vision_radius:
        return True

# collision check
def lineIntersection(traj_1, traj_2):
    p1_start = Point2D(traj_1[0][0], traj_1[1][0])
    p1_end = Point2D(traj_1[0][-1], traj_1[1][-1])
    p2_start = Point2D(traj_2[0][0], traj_2[1][0])
    p2_end = Point2D(traj_2[0][-1], traj_2[1][-1])
    intersect = Point2D(0,0)
    p0_x = p1_start.x
    p0_y = p1_start.y
    p1_x = p1_end.x
    p1_y = p1_end.y
    p2_x = p2_start.x
    p2_y = p2_start.y
    p3_x = p2_end.x
    p3_y = p2_end.y

    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y))/(-s2_x * s1_y + s1_x * s2_y)
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x))/(-s2_x * s1_y + s1_x * s2_y)

    if s >= 0 and s <= 1 and t >=0 and t <= 1:
        # collision detected
        intersect.x = p0_x + (t * s1_x)
        intersect.y = p0_y + (t * s1_y)
        print("!!COLLISION!!")
        return True
    
    return False  # no collision

# move for actual update

# for time being, i am updating the next position from the future trajectory point 
# directly. in real world, we will get it from pose after the detection part
# collect the info from vehicle[i].twist.twist.linear._
def move(x, y, v, dt_m, path):
    # find the closest index from the curve and compute theta between those points
    # shift the vehicle along that direction to get the modified points

    ind_closest = closest_point_ind(path, x, y)
    # Determine the indices of the 2 closest points
    if ind_closest < len(path):
        # Check if we are at the end of the segment
        if ind_closest == len(path) - 1:
            use_previous = True
        elif ind_closest == 0:
            use_previous = False
        else:
            dist_prev = distance(path[ind_closest-1].x, path[ind_closest-1].y, x, y)
            dist_next = distance(path[ind_closest+1].x, path[ind_closest+1].y, x, y)

            if dist_prev <= dist_next:
                use_previous = True
            else:
                use_previous = False

        # Get the 2 points
        if use_previous:
            p1 = Point2D(path[ind_closest - 1].x, path[ind_closest - 1].y)
            p2 = Point2D(path[ind_closest].x, path[ind_closest].y)
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)

        # Get the point in the local coordinate with center p1
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
    new_x = x + v*np.cos(theta)*dt_m
    new_y = y + v*np.sin(theta)*dt_m

    return [new_x, new_y]

# future waypoints
def PredictTrajectoryVehicles(init_x, init_y, path, s_map, v):    # msg_vehicles -> traffic_msg/PredictionArray
    s, d, _ = get_frenet(init_x, init_y, path, s_map)
    future_x = []
    future_y = []
    for t in range(np_m):
        if t < interp_back_path:
            d_val = d - ((t*d) / interp_back_path)
            new_x, new_y, _ = get_xy(s+v*dt_m*t, d_val, path, s_map)
        else:
            new_x, new_y, _ = get_xy(s+v*dt_m*t, 0, path, s_map)
        future_x.append(new_x)
        future_y.append(new_y)
    return future_x, future_y

# getting the future trajectory
def get_future_trajectory(x, y, current_waypoint, v, id):
    # return the future trajectory of the vehicle
    lane_line_list, lane_s_map = get_lane_and_s_map(x, y)
    future_x, future_y = PredictTrajectoryVehicles(current_waypoint[0], current_waypoint[1], lane_line_list, lane_s_map, v)
    curr = move(current_waypoint[0], current_waypoint[1], v, dt_m, lane_line_list)
    # update these waypoints as ros messages -> geometry_msgs.pose.position
    # later provide this information on ros traffic messages

    yaw = np.pi/4
    out = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(out[0], out[1], out[2], out[3])
    pose = geometry_msgs.Pose(Point(curr[0], curr[1], 0), orientation)
    linear = Vector3(v_ego*math.cos(yaw), v_ego*math.sin(yaw), 0)
    angular = Vector3(0, 0, yaw)
    twist = geometry_msgs.Twist(linear, angular)
    Vehicle.vehicle_dynamics[id] = [pose, twist, s, d, past_waypoints, [future_x, future_y]]
    return [future_x, future_y], pose, twist

# get the s map and lane info
def get_lane_and_s_map(x1, y1):
    pose_arr = []
    # x_g, y_g = get_spline(current_waypoint[0],destination_waypoint[0],current_waypoint[1],destination_waypoint[1],np.pi/2,np.pi/4)
    lane_route = []
    for i in range(len(x1)):
        lane_route.append([x1[i], y1[i]])
    
    for i in range(len(lane_route)-1):
        point = Point(lane_route[i][0], lane_route[i][1])
        # replace this by actual yaw of the vehicle maybe
        yaw = math.atan2((lane_route[i+1][1]-lane_route[i][1]),(lane_route[i+1][0]-lane_route[i][0]))
        out = quaternion_from_euler(0,0,yaw)
        quat = Quaternion(out[0], out[1], out[2], out[3])
        poses = PoseStamped(std_msgs.Header(), Pose(point, quat))
        pose_arr.append(poses)
    # adding the last point
    pose_arr.append(PoseStamped(std_msgs.Header(), Pose(Point(lane_route[-1][0], lane_route[-1][1]), Quaternion(out[0], out[1], out[2], out[3]))))
    path_route = Path(std_msgs.Header(), pose_arr)
    lane_line_list, lane_s_map = path_to_list(path_route)

    return lane_line_list, lane_s_map

# generating straight lanes
def get_straight(x0, y0, x1, y1, theta, steps=100):
    x = np.linspace(x0, x1, steps)
    y = np.linspace(y0, y1, steps)

    # horizontal road
    if theta == 0:
        y_boundary_1 = [a-(width/2) for a in y]
        y_boundary_2 = [a+(width/2) for a in y]
        return x, y, y_boundary_1, y_boundary_2
    # vertical road
    else:
        x_boundary_1 = [a-(width/2) for a in x]
        x_boundary_2 = [a+(width/2) for a in x]
        return x, y, x_boundary_1, x_boundary_2

# generate spline lanes
def get_spline(x0, x1, y0, y1, theta0, theta1, steps=100): 
    # change the format of input to the function to point
	t = np.linspace(0, 1, steps) 

	dx0 = np.cos(theta0) 
	dy0 = np.sin(theta0)
	dx1 = np.cos(theta1) 
	dy1 = np.sin(theta1)

	t0 = 0
	t1 = 1

	Ax = np.asarray([[1, t0,   t0**2,   t0**3],  # x  @ 0
					[0, 1,  2*t0,    3*t0**2],  # x' @ 0
					[1, t1,   t1**2,   t1**3],  # x  @ 1
					[0, 1,  2*t1,    3*t1**2]]) # x' @ 1

	X = np.asarray([x0, dx0, x1, dx1]).transpose()
	bx = np.linalg.solve(Ax, X)

	Ay = np.asarray([[1, t0,   t0**2,   t0**3],  # x  @ 0
					[0, 1,  2*t0,    3*t0**2],  # x' @ 0
					[1, t1,   t1**2,   t1**3],  # x  @ 1
					[0, 1,  2*t1,    3*t1**2]]) # x' @ 1
	Y = np.asarray([y0, dy0, y1, dy1]).transpose()
	by = np.linalg.solve(Ay, Y)

	x = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(),bx)
	y = np.dot(np.vstack([np.ones_like(t), t, t**2, t**3]).transpose(),by)

	return x, y

# main function
if __name__ == '__main__':
    width = 2                       # lane width
    interp_back_path = 20          # interpolate back to path after this # of steps
    plan_t_m = 1                   # planning horizon
    dt_m = 0.1                      # time step update
    np_m = int(plan_t_m/dt_m)       # number of future waypoints
    vision_radius = 1               # register vehicles which are within this radius of the ego vehicle

    # path
    x, y = get_spline(0,3,0,3,np.pi/2,np.pi/4)
    x_,y_ = get_spline(3,6,3,0,np.pi/2,np.pi/4)
    x1 = x + x_
    y1 = y + y_
    plt.plot(x1, y1, 'g')

    x, y = get_spline(0,3,0,3,-np.pi/2,-np.pi/4)
    x_,y_ = get_spline(3,6,3,0,-np.pi/2,-np.pi/4)
    x2 = x + x_
    y2 = y + y_
    plt.plot(x2, y2, 'k')

    # registering the ego vehicle 
    position = Point(3.04, 3.05, 0)
    yaw = np.pi/4                                             # change
    out = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(out[0], out[1], out[2], out[3])
    v_ego = 0.9                                         # obtain from perception || vehicles.twist.twist.linear
    linear = Vector3(v_ego*math.cos(yaw), v_ego*math.sin(yaw), 0)
    angular = Vector3(0, 0, yaw)
    ego_pose = geometry_msgs.Pose(position, orientation)
    ego_twist = geometry_msgs.Twist(linear, angular)
    s = 0
    d = 0
    past_waypoints = None                               # record waypoints  # think how to implement this
    future_waypoints_ego, _, _ = get_future_trajectory(x1, y1, [position.x, position.y], v_ego, "ego")
    
    ego = Vehicle(ego_pose, ego_twist, s, d, past_waypoints, future_waypoints_ego, "ego")
    ego.register_ego(ego)

    # other vehicles 
    position = Point(4.5, 2.8, 0)
    yaw = np.pi/4                                             # change
    out = quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(out[0], out[1], out[2], out[3])
    v_veh = 0.5                                         # obtain from perception || vehicles.twist.twist.linear
    linear = Vector3(v_ego*math.cos(yaw), v_ego*math.sin(yaw), 0)
    angular = Vector3(0, 0, yaw)
    veh_pose = geometry_msgs.Pose(position, orientation)
    veh_twist = geometry_msgs.Twist(linear, angular)
    s = 0
    d = 0
    past_waypoints = None                               # record waypoints  # think how to implement this
    future_waypoints_veh, _, _ = get_future_trajectory(x2, y2, [position.x, position.y], v_veh, "veh1")
    
    veh = Vehicle(veh_pose, veh_twist, s, d, past_waypoints, future_waypoints_veh, "veh1")

    future_traj_1 = future_waypoints_ego
    future_traj_2 = future_waypoints_veh

    while not lineIntersection(future_traj_1, future_traj_2):
        veh.register_vehicle(ego, veh)
        future_traj_1, ego_pose, ego_twist = get_future_trajectory(x1, y1, [ego_pose.position.x, ego_pose.position.y],v_ego, "ego")
        future_traj_2, veh_pose, veh_twist = get_future_trajectory(x2, y2, [veh_pose.position.x, veh_pose.position.y], v_veh, "veh1")
        ego = Vehicle(ego_pose, ego_twist, s, d, past_waypoints, future_traj_1, "ego")
        veh = Vehicle(veh_pose, veh_twist, s, d, past_waypoints, future_traj_2, "veh1")
        # print(Vehicle.count)
        # for id, val in Vehicle.vehicle_dynamics.items():
        #     print(id, val[0].position.x)
        plt.plot(future_traj_1[0], future_traj_1[1], 'r--')
        plt.plot(ego_pose.position.x, ego_pose.position.y, 'b*')
        plt.plot(future_traj_2[0], future_traj_2[1], 'g--')
        plt.plot(veh_pose.position.x, veh_pose.position.y, 'b*')
        plt.pause(0.2)
    plt.show()