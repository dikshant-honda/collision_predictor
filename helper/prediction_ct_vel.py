import numpy as np
from geometry_utils import *
from frenet import *
from py_msgs.traffic_msgs import *
from py_msgs.geometry_msgs import *
from py_msgs.nav_msgs import *
from std_msgs import *
from lane_publisher import *

# last updated on: 2023/03/15
# not in use anymore

'''
USAGE:
python based script for predicting future trajectory in Frenet coordinate system based on constant velocity assumption
'''

'''
=> variable names
1. lane center callback: determining the center of the lanes
2. prediction array: type of traffic message
3. target_lane_m: stores target lane id
4. plan_t: plan horizon length in sec and dt_m is the time increment
5. kADLane: id used for AV ego lane
6. kTargetCrossTraffic: id used as the target lane in a cross traffic scenario
7. sorted_vehicles: vector of vehicle states sorted from front to back in direction of traffic
8. interp_back_path: how many meters to return to path
9. lifetime_id: id for the agent
'''

def PedestrianCallback():
    # get the pedestrian data in the form of pedestrian state array and store to 
    # pedestrain_vec_m variable
    pass

def LanesPerceptionCallback():
    # get the lane information and store 2 things
    # map_car_lanes_m: map for each car it tells how many lanes it is in
    # map_vehicles_lanes_m: map for correspoding lane id, ig
    pass

def LanesCenterCallback(CenterLanes):
    # getting the lane centers and storing them to map_lanes_frenet_frame
    map_lanes_frenet_m.clear()
    for l in range(len(CenterLanes.ids)):
        path = [Point2D()]*CenterLanes[l].path.poses.size()
        for i in range(CenterLanes[l].path.poses.size()):
            path[i] = Point2D(CenterLanes[l].path.poses[i].pose.position.x,
                              CenterLanes[l].path.poses[i].pose.position.y)
        map_lanes_frenet_m[CenterLanes.ids[l]] = ToFrenet(path)  ## check

def TimerCallback():
    # define all the timeout and failure conditions
    # store lane and cars future prediction messages
    pass

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
        map_lanes_frenet_m[project_lane].ToFrenet(p_xy, p_sd)

        # int_traj.trajectory_estimated.waypoints = np.resize(int_traj.trajectory_estimated.waypoints, np_m)
        # int_traj.trajectory_uncertainity.waypoints = np.resize(int_traj.trajectory_uncertainity.waypoints, np_m)
        interp_back_path = 20
        for t in range(np_m):
            pred_xy = Point2D()
            road_dir = 0
            if t < interp_back_path:
                d_val = p_sd.d - t*p_sd.d / interp_back_path
                map_lanes_frenet_m[project_lane].ToCartersian(
                    Point_Frenet(p_sd.s+v*dt_m*t, d_val), pred_xy, road_dir)
            else:
                map_lanes_frenet_m[project_lane].ToCartesian(
                    Point_Frenet(p_sd.s+v*dt_m*t, 0), pred_xy, road_dir)
            
if __name__ == '__main__':
    lanes_perception_sub_m = None
    pedestrian_sub_m = None
    lanes_center_sub_m = None
    target_lane_sub_m = None
    lanes_prediction_pub_m = None       # publish
    pedestrian_prediction_pub_m = None  # publish
    all_cars_pub_m = None               # publish
    
    # functions to be subscribed
    LanesPerceptionCallback()
    PedestrianCallback()
    LanesCenterCallback()
    # TargetLaneCallback()

    lanes_perception = None # region/lanes_perception
    lanes_perception_sub_m = LanesPerceptionCallback(lanes_perception)
    lanes_center = lane_publisher.publish_all_lanes()
    # timer = None # ros time
    # timer_compute_m = TimerCallback(timer)

    # planning horizon parameters
    plan_t_m = 7
    dt_m = 0.1
    np_m = round(plan_t_m / dt_m) + 1

    # prediction variables
    max_speed_agts_m = 35

    # vehicle_variables
    map_vehicles_lane_m = {}    # { (int) key : (list(traffic_message.VehicleState)) value  }
    map_lanes_frenet_m = {}     # { (int) key : (geometry.Frenet) value 
    map_car_lanes_m = {}        # { (int) key : (list) value

    # target_lane_check
    target_lane_m = -100
    prev_target_lane_m = -200