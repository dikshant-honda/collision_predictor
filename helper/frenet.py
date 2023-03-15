import numpy as np
import math
import bisect
from geometry_utils import *

# last updated on: 2023/03/15

'''
USAGE:
helper functions for converting to frenet from cartesian and vice-versa
'''

def global_to_local(ref_orig, orientation, p):
    # converting the global frame to local frame for helping frenet transform
    delta = Point2D(p.x - ref_orig.x, p.y - ref_orig.y)

    s = math.sin(-orientation)
    c = math.cos(-orientation)

    out = Point2D(delta.x * c - delta.y * s,
    delta.x * s + delta.y * c)

    return out

def local_to_global(center, theta, p):
    # converting the local frame to global frame for bringing back to cartesian coordinates
    s = math.sin(theta)
    c = math.cos(theta)

    out = Point2D(p.x * c - p.y * s + center.x, p.x * s + p.y * c + center.y)

    return out

def path_to_list(nav_path):
    # converting ther nav_path message type to list for ease in accessibility
    # generating s_map from the start point till end point for transforms
    path_list = []
    distance_acum = 0.0
    s_map = []
    prev_p = None
    for pose in nav_path.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        path_list.append(Point2D(x, y))
        if prev_p != None:
            distance_acum += distance(prev_p.x, prev_p.y, x, y)
        s_map.append(distance_acum)
        prev_p = Point2D(x, y)
    return path_list, s_map

def distance(x1, y1, x2, y2):
    return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

def closest_point_ind(path, x, y):
    # finding the closest index on path from point(x,y)
    index = 0
    closest_index = 0
    min_dist = 10000.0
    for p in path:
        dist = distance(p.x, p.y, x, y)
        if dist < min_dist:
            min_dist = dist
            closest_index = index
        index += 1
    return closest_index

# Transform from Cartesian x,y coordinates to Frenet s,d coordinates
def get_frenet(x, y, path, s_map):
    if path == None:
        print("Empty map. Cannot return Frenet coordinates")
        return 0.0, 0.0, False

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
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest

        # Get the point in the local coordinate with center p1
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
        local_p = global_to_local(p1, theta, Point2D(x,y))

        # Get the coordinates in the Frenet frame
        p_s = s_map[prev_idx] + local_p.x
        p_d = local_p.y

    else:
        print("Incorrect index")
        return 0.0, 0.0, False

    return p_s, p_d, True

# adding on theta to get the yaw of the vehicle
def get_frenet_with_theta(x, y, path, s_map):
    if path == None:
        print("Empty map. Cannot return Frenet coordinates")
        return 0.0, 0.0, 0.0, False

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
            prev_idx = ind_closest - 1
        else:
            p1 = Point2D(path[ind_closest].x, path[ind_closest].y)
            p2 = Point2D(path[ind_closest + 1].x, path[ind_closest + 1].y)
            prev_idx = ind_closest

        # Get the point in the local coordinate with center p1
        theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
        local_p = global_to_local(p1, theta, Point2D(x,y))

        # Get the coordinates in the Frenet frame
        p_s = s_map[prev_idx] + local_p.x
        p_d = local_p.y

    else:
        print("Incorrect index")
        return 0.0, 0.0, 0.0, False

    return p_s, p_d, theta, True

# Transform from Frenet s,d coordinates to Cartesian x,y
def get_xy(s, d, path, s_map):

    if path == None or s_map == None:
        print("Empty path. Cannot compute Cartesian coordinates")
        return 0.0, 0.0, False

    # If the value is out of the actual path send a warning
    if s < 0.0 or s > s_map[-1]:
        if s < 0.0:
            prev_point = 0
        else:
            prev_point = len(s_map) -2
    else:
        # Find the previous point
        idx = bisect.bisect_left(s_map, s)
        prev_point = idx - 1

    p1 = path[prev_point]
    p2 = path[prev_point + 1]

    # Transform from local to global
    theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
    p_xy = local_to_global(p1, theta, Point2D(s - s_map[prev_point], d))

    return p_xy.x, p_xy.y, True

def dist_to_line(p1, p2, p):
    p1 = np.array(p1)
    p2 = np.array(p2)
    p = np.array(p)
    d = np.cross(p2-p1,p-p1)/np.linalg.norm(p2-p1)
    return abs(d)

'''
past functions converted from cpp(mpqp algorithm) , not required anymore!
'''
path_m = Point2D(0,0)
path_s_m = []
path_theta_m = []

def CreateSPath(spec_origin, origin):
    if len(path_m) < 2: return

    path_s_m = np.resize(path_s_m, len(path_m))
    path_theta_m = np.resize(path_theta_m, len(path_m))

    if not spec_origin:
        path_s_m[0] = 0
        for i in range(1, len(path_s_m)):
            path_s_m[i] = distance(path_m[i], path_m[i-1]) + path_s_m[i-1]
            dir_vec = path_m[i] - path_m[i-1]
            path_theta_m[i-1] = math.atan2(dir_vec.y, dir_vec.x)
    
        path_theta_m[len(path_theta_m)-1] = path_theta_m[len(path_theta_m)-2]
    else:
        ind_closest = findClosestIndex2D(path_m, origin)
        path_s_m[ind_closest] = 0

        for i in range(ind_closest-1, -1, -1):
            path_s_m[i] = path_s_m[i+1] - distance(path_m[i], path_m[i+1])
            dir_vec = path_m[i+1] - path_m[i]
            path_theta_m[i] = math.atan2(dir_vec.y, dir_vec.x)

        for i in range(ind_closest + 1, len(path_m)):
            path_s_m[i] = distance(path_m[i], path_m[i-1]) + path_s_m[i-1]
            dir_vec = path_m[i] - path_m[i-1]
            path_theta_m[i-1] = math.atan2(dir_vec.y, dir_vec.x)
        path_theta_m[len(path_theta_m)-1] = path_theta_m[len(path_theta_m)-2]

def SetPath(path, spec_origin, origin):
    if len(path) < 2:
        path_m.clear() 
        path_s_m.clear()
        print("Frenet path needs at least 2 points")
        return
    path_m = path
    CreateSPath(spec_origin, origin)

def findClosestIndex2DWithDirection(point_input, theta):
    min_distance = np.inf
    ind_closest = -1
    for i in range(0, len(path_m)):
        aux_angle = math.atan2(np.sin(path_theta_m[i]-theta), np.cos(path_theta_m[i]-theta))
        if aux_angle < -3*np.pi/4 or aux_angle > 3*np.pi/4:
            continue
        distance_val = distance(point_input, path_m[i])
        if distance_val < min_distance:
            min_distance = distance_val
            ind_closest = i
    
    return ind_closest

def ToFrenet(p_xy, p_sd, road_dir, closest):
    if len(path_m) < 2:
        print("Path needs at least 2 points. Cannot compute Frenet")
        p_sd.s = -np.inf
        p_sd.d = -np.inf
        return False
    
    if closest == -1:
        closest = findClosestIndex2D(path_m, p_xy)
    
    closest_ind = 0
    prev_idx = 0
    if closest < 0:
        print("Wrong closest index. Cannot compute Frenet")
        p_sd.s = -np.inf
        p_sd.d = -np.inf
        return False
    else:
        closest_ind = closest

    use_previous = False

    if closest_ind < len(path_m):
        if closest_ind == len(path_m)-1:
            use_previous = True
        elif closest == 0:
            use_previous = False
        else:
            dist_prev = distance(path_m[closest_ind - 1], p_xy)
            dist_next = distance(path_m[closest_ind + 1], p_xy)
            if dist_prev <= dist_next:
                use_previous = True
            else:
                use_previous = False
        
        p1 = Point2D()
        p2 = Point2D()
        if use_previous:
            p1.x = path_m[closest_ind - 1].x
            p1.y = path_m[closest_ind - 1].y
            p2.x = path_m[closest_ind].x
            p2.y = path_m[closest_ind].y
            prev_idx = closest_ind - 1
        else:
            p1.x = path_m[closest_ind].x
            p1.y = path_m[closest_ind].y
            p2.x = path_m[closest_ind + 1].x
            p2.y = path_m[closest_ind + 1].y
            prev_idx = closest_ind
        
        road_dir = math.atan2(p2.y - p1.y, p2.x - p1.x)
        local_p = globalToLocal(p1, road_dir, p_xy)

        p_sd.s = path_s_m[prev_idx] + local_p.x
        p_sd.d = local_p.y
    else:
        print("incorrect index")
        p_sd.s = -np.inf
        p_sd.d = -np.inf
        road_dir = 100
        return False
    return True

def ToFrenetDirectional(p_xy, theta, p_sd, road_dir):
    closest = findClosestIndex2DWithDirection(p_xy, theta)
    if closest == -1:
        print("Can't find index in frenet directional")
        return False
    return ToFrenet(p_xy, p_sd, road_dir, closest)

def ToCartesian(p_sd, p_xy, road_dir):
    if len(path_m) < 2:
        print("Path needs at least 2 points. Cannot compute Frenet")
        p_xy.x = -np.inf
        p_xy.y = -np.inf
        return False

    prev_point_ind = 0

    if p_sd.s <= path_s_m[0] or p_sd.s >= path_s_m[-1]:
        if p_sd.s < path_s_m[0]:
            prev_point_ind = 0
        else:
            prev_point_ind = len(path_s_m) - 2
    else:
        it = bisect.bisect_left(path_s_m, p_sd.s)
        prev_point_ind = it - path_s_m[0] - 1
    
    p1 = Point2D()
    p2 = Point2D()
    p1.x = path_m[prev_point_ind].x
    p1.y = path_m[prev_point_ind].y
    p2.x = path_m[prev_point_ind+1].x
    p2.y = path_m[prev_point_ind+1].y

    road_dir = math.atan2(p2.y - p1.y, p2.x - p1.x)
    p_xy = localToGlobal(p1, road_dir,Point2D(p_sd.s - path_s_m[prev_point_ind], p_sd.d))
    return True