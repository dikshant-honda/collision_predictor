import numpy as np
import math
import sys
from geometry_utils import *

path_m = Point2D(0,0)
path_s_m = []
path_theta_m = []

def CreateSPath(spec_origin, origin):
    if path_m.size < 2: return

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

        for i in range(ind_closest, len(path_m)):
            path_s_m[i] = distance(path_m[i], path_m[i-1]) + path_s_m[i-1]
            dir_vec = path_m[i] - path_m[i-1]
            path_theta_m[i-1] = math.atan2(dir_vec.y, dir_vec.x)
        path_theta_m[len(path_theta_m)-1] = path_theta_m[len(path_theta_m)-2]

def SetPath(path, spec_origin, origin):
    if path.size < 2:
        path_m.clear() 
        path_s_m.clear()
        print("Frenet path needs at least 2 points")
        return
    path_m = path
    CreateSPath(spec_origin, origin)

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

