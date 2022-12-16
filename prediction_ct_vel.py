import numpy as np
import math
import bisect
from geometry_utils import *
from frenet import *

# const
kADLane = 0
kTargetCrossTraffoc = 100

# simulation variables
plan_t_m = None
dt_m = None
np_m = None
target_lane_m = None
count_prev_lane_m = None
lane_switch_case_m = False

# prediction variables
max_speed_agts_m = None

# vehicle_variables
map_lanes_frenet_m = None
map_car_lanes_m = None

def PredictTrajectoryVehicles(sorted_vehicles, lane_id, msg_vehicles):
    n_cars = len(sorted_vehicles)
    for k in range(n_cars):
        car_id = sorted_vehicles[k].lifetime_id         ### check

        # projection lane selection
        project_lane = 0
        lanes_for_car = map_car_lanes_m[car_id]         ### check

        msg_pred = None                      ### check
        msg_pred.dt = dt_m
        # msg_pred.lane_id = lane_id
        msg_pred.agent_id = car_id
        msg_pred.length = sorted_vehicles[k].length
        msg_pred.width = sorted_vehicles[k].width
        # int_traj = IntentionTrajectory()
        # int_traj.trajectory_probability = 1.0

        p_xy = Point2D()
        p_xy.x = sorted_vehicles[k].x
        p_xy.y = sorted_vehicles[k].y

        v = min(np.hpot(sorted_vehicles[k].v.x, sorted_vehicles[k].v.y), max_speed_agts_m)

        p_sd = Point_Frenet()
        map_lanes_frenet_m[project_lane].ToFrentet(p_xy, p_sd)

        # int_traj.trajectory_estimated.waypoints = np.resize(int_traj.trajectory_estimated.waypoints, np_m)
        # int_traj.trajectory_uncertainity.waypoints = np.resize(int_traj.trajectory_uncertainity.waypoints, np_m)
        interp_back_path = 20
        for t in range(np_m):
            pred_xy = Point2D()
            road_dir = 0
            if t < interp_back_path:
                d_val = p_sd.d - t*p_sd.d / interp_back_path
                map_lanes_frenet_m[project_lane].ToCartersian(
                    Point_Frenet(p_sd.s+v*dt_m*t, d_val)
                )
