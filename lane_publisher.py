import numpy as np
from geometry_utils import *
from frenet import *
from traffic_msgs import *
from geometry_msgs import *
from nav_msgs import *
from std_msgs import *
from tf import *
from collections import namedtuple

LaneInfo = namedtuple('LaneInfo', ['start_wp', 'target_wp', 'merging_wp','speed_limit'])

class lane_publisher:
    def __init__(self):
        self.lanes_center = CenterLanes()
        self.target_lane_id = 1   # change id of type int
        self.lane_info_dict = {}

        self.target_lane_key = None
        goal_pt_param = None     # give the goal point of the vehicle, len = 3

        lane_ids = None         # get param from scenario/lanes
        if not lane_ids:
            print("Missing lane information")
        
        self.max_lat_acc = 2.0

        for lane_vals in lane_ids:
            key_st_pt = "scenario/start_point_lane" + str(lane_vals)
            key_tg_pt = "scenario/target_point_lane" + str(lane_vals)
            key_mg_pt = "scenario/merging_point_lane" + str(lane_vals)
            # key_mg_s_pt = "scenario/merge_start_point_lane" + str(lane_vals)
            # key_mg_e_pt = "scenario/merge_end_point_lane" + str(lane_vals)
            # key_sl = "scenario/speed_limit_lane" + str(lane_vals)
            # key_al = "scenario/acc_limits_lane" + str(lane_vals)
            # key_jl = "scenario/jerk_limits_lane" + str(lane_vals)
            # key_sb_pt = "scenario/stop_bar_point_lane" + str(lane_vals)

            start_pt = key_st_pt        # rospy.get_param
            if not start_pt:
                NameError("Missing lane '{}' start point".format(lane_vals))
                exit()

            target_pt = key_tg_pt       # rospy.get_param
            if not target_pt:
                NameError("Missing lane '{}' target point".format(lane_vals))
                exit()

            merge_pt = key_mg_pt        # rospy.get_param
            if not merge_pt:
                NameError("Missing lane '{}' merging point".format(lane_vals))
                exit()

            # merging start and end points left for time being

            # can be specified using ros-params
            spd_lim_val = 5.0
            # acc_min = -2.0
            # acc_max = 2.0
            # jrk_min = -1.5
            # jrk_max = 1.5
            self.lane_info_dict[lane_vals] = LaneInfo(start_wp=start_pt, target_wp=target_pt, merging_wp=merge_pt, speed_limit=spd_lim_val)

            # if lane_vals == target_lane_val:
            #     # self.merging_wp_target = merge_wp
            #     self.merging_wp_target = merge_end_wp
            #     self.merging_s_set = False

        def on_odom(self, odom):   # complete this function
            if not self.lane_fixed:
                self.lane_fixed = True
                self.reroute()
            
            # figure out AD car lane
            if self.lanes_dict:
                ## define scenarios in future
                if self.ad_current_lane != self.ad_past_lane:
                    self.ad_past_lane = self.ad_current_lane
                    self.lane_published = False
            
            if not self.lane_published:
                self.publish_all_lanes()


        # AD position
        self.ego_odom = on_odom()
        self.ad_current_pose = None
        self.ad_current_lane = -100
        self.ad_past_lane = -100
        self.passed_merging = False
        self.lanes_published = False
        self.lane_fixed = False
        self.lanes_dict = {}
        self.lanes_frenet = {}

    def reset(self):
        self.lanes_published = False
        self.lane_fixed = False
        self.lanes_dict = {}
        self.lanes_frenet = {}
        self.ad_current_pose = None
        self.ad_current_lane = -100
        self.ad_past_lane = -100
        self.passed_merging = False

    def reroute(self):
        """
        Triggers a rerouting
        """
        if self.ego_vehicle is None:
            # no ego vehicle, remove route if published
            self.lanes_dict = {}
        else:
            self.calculate_route()

    def find_ego_vehicle(self):
        '''
        use it later for switching between ego vehicles
        '''
        pass

    def calculate_route(self):
        grp = None
        ## design global route planner (grp) for tracing route
        ## design a global route planner for traffic participants
        # dao = GlobalRoutePlannerDAO(self.map, sampling_resolution=1.0)
        # grp = GlobalRoutePlanner(dao)
        # grp.setup()

        self.lanes_dict = {}
        self.lanes_frenet = {}
        self.target_lane_key = None

        # For each lane find the route and save it in the dictionary
        for key, value in self.lane_info_dict.items():
             # find the center lane for each of the lanes
            lane_route = grp.trace_route(value.start_wp.transform.location, value.target_wp.transform.location)
            lane_route.append([value.target_wp, None])

            print("Lane ID: %d, Length lane: %d" % (key, len(lane_route)))

            #  Transform to path
            path_route = Path()
            prev_wp = None
            for wp in lane_route:
                if prev_wp is not None:
                    dist_wp = distance(wp[0].transform.location.x, wp[0].transform.location.y,
                                       prev_wp[0].transform.location.x, prev_wp[0].transform.location.y)
                    if dist_wp < 0.1:
                        continue

                prev_wp = wp

                pose = PoseStamped()
                pose.pose.position.x = wp[0].transform.location.x
                pose.pose.position.y = -wp[0].transform.location.y
                pose.pose.position.z = wp[0].transform.location.z

                # design class for tf
                quaternion = quaternion_from_euler(
                    0, 0, -math.radians(wp[0].transform.rotation.yaw))
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                path_route.poses.append(pose)
            
            self.lanes_dict[key] = path_route
            lane_line_list, lane_s_map = path_to_list(path_route)
            self.lanes_frenet[key] = [lane_line_list, lane_s_map]

            # Find in which lane the global target point is and save the key
            s, d, conv = get_frenet(
                self.target_pt_lane[0], -self.target_pt_lane[1], lane_line_list, lane_s_map)
            if abs(d) < value.target_wp.lane_width / 2:
                self.target_lane_key = key
                print("Target lane in global id: %d" % key)

    def publish_all_lanes(self):
        msg_center_lanes = CenterLanes()
        msg_center_lanes.header.frame_id = "map"
        already_merged = False
        if self.lanes_dict:
            for key, value in self.lane_info_dict.items():
                center_line_aux = PathWithSpeed()
                center_line_aux.path = self.lanes_dict[key]
                center_line_aux.apth = cleanup_close_points(center_line_aux.path)   # explore this function
                center_line_aux.curvature_speeds = compute_speed_based_on_curvature(center_line_aux.path, self.max_lat_acc, value.speed_limit)
                msg_center_lanes.center_lines.append(center_line_aux)

                msg_center_lanes.lanes_width.append(value.target_wp.lane_width) # check

                msg_center_lanes.speed_limits.append(value.speed_limit) # check

                pt_merging = Point()
                pt_merging.x = value.merging_wp.transform.location.x
                pt_merging.y = -value.merging_wp.transform.location.y
                pt_merging.z = value.merging_wp.transform.location.z
                msg_center_lanes.crossing_ad_points.append(pt_merging)
        
            self.lanes_published = True
        
        self.lanes_center = msg_center_lanes
