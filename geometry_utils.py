import numpy as np
import sys
sys.path.append('../')
from msg.geometry_msgs import Pose2D
from msg.nav_msgs import Path
from typing import List
from scipy.signal import butter, filtfilt

class Point2D:
    def __init__(self,x_init=0,y_init=0):
        self.x = x_init
        self.y = y_init

    def shift(self, x, y):
        self.x += x
        self.y += y

    def __repr__(self):
        return "".join(["Point(", str(self.x), ",", str(self.y), ")"])

class Point3D():
    def __init__(self,x=0,y=0,z=0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return "(%i, %i, %i)"%(self.x,self.y,self.z)

class Point_Frenet():
    def __init__(self, s_init, d_init):
        self.s = s_init
        self.d = d_init

def distance(p1, p2):
    return np.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

def perpDotProduct(a, b, c):
    return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)

def dotProduct(a, b):
    return a.x * b.x + a.y * b.y

def lineIntersection(p1_start, p1_end, p2_start, p2_end, intersect):
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
        return True
    
    return False  # no collision

def isProjectedPointOnLine(line_start, line_end, p):
    e1 = Point2D(line_end.x - line_start.x, line_end.y - line_start.y)
    e2 = Point2D(p.x - line_start.x, p.y - line_start.y)
    recArea = dotProduct(e1, e1)
    val = dotProduct(e1, e2)

    return val >= 0 and val <= recArea

def getProjectedPointOnLine(line_start, line_end, p):
    e1 = Point2D(line_end.x - line_start.x, line_end.y - line_start.y)
    e2 = Point2D(p.x - line_start.x, p.y - line_start.y)
    val = dotProduct(e1, e2)
    len2 = e1.x * e1.x + e1.y * e1.y
    res = Point2D((line_start.x + (val * e1.x) / len2),
    (line_start.y + (val * e1.y) / len2))

    return res

def distanceToLine(start, end, point):
    normalLength = np.hypot(end.x - start.x, end.y - start.y)
    dist = ((point.x - start.x) * (end.y - start.y) - (point.y - start.y) * (end.x - start.x)) / normalLength
    return np.fabs(dist)

##################### 3D point functions  ###################################

def distance3D(p1, p2):
    return np.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z))

def judgeDirection(p1, p2, p):
    A = -(p2.y - p1.y)
    B = p2.x - p1.x
    C = -(A * p1.x + B * p1.y)
    D = A * p.x + B * p.y + C
    if D <= 0.00001 and D >= -0.00001:  return 0
    if D > 0:   return 1
    if D < 0:   return -1

def isPointInRect(r1, p):
    i = 0
    i += judgeDirection(r1.p[0], r1.p[1], p)
    i += judgeDirection(r1.p[1], r1.p[2], p)
    i += judgeDirection(r1.p[2], r1.p[3], p)
    i += judgeDirection(r1.p[3], r1.p[0], p)
    if i == (-4) or i == (-3):  return True
    return False

def perpDotProduct(a, b, c):
    return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x)

def isInRange2D(a, b, range):
    return distance(a, b) <= range

def isSamePoint2D(a, b, tolerance):
    return distance(a, b) <= tolerance

############################################################################

def findClosestIndex2D(vect_pts, point_input):
    min_distance = np.inf
    ind_closest = -1

    for i in range(len(vect_pts)):
        distance_val = distance(point_input, vect_pts[i])
        if distance_val < min_distance:
            min_distance = distance_val
            ind_closest = i
    
    return ind_closest

def rotate(inp, theta):
    out = Point2D()
    s = np.sin(theta)
    c = np.cos(theta)
    out.x = inp.x*c - inp.y*s
    out.y = inp.x*s + inp.y*c

    return out

def globalToLocal(center, theta, p):
    delta = Point2D()
    delta.x = p.x - center.x
    delta.y = p.y - center.y 
    return rotate(delta, -theta)

def localToGlobal(center, theta, p):
    out = rotate(p, theta)
    out.x += center.x
    out.y += center.y
    return out

def judgeDirection(p1, p2, p):
    A = -(p2.y - p1.y)
    B = p2.x - p1.x
    C = -(A * p1.x + B * p1.y)
    D = A * p.x + B * p.y + C

    if D <= 0.00001 and D >= -0.00001:  return 0
    if D > 0:   return 1
    if D <0:    return -1

##################################################################

def compute_speed_based_on_curvature(path: Path, lat_acc_max: float, speed_limit: float) -> List[float]:
    """
    Compute speed limit based on curvature for each point
    Args:
            center_line: Center line path
            lat_acc_max: Maximum lateral acceleration [m/s2]
            speed_limit: Speed limits for that center line [m/s]
    Returns:
            List of speed limits for each point
    """

    curvature = np.zeros(len(path.poses), dtype=float)
    for i in range(len(path.poses)):
        if i >= len(path.poses)-2:
            curvature[i] = curvature[i-1]
        else:
            a = np.array(
                (path.poses[i].pose.position.x, path.poses[i].pose.position.y))
            b = np.array(
                (path.poses[i+1].pose.position.x, path.poses[i+1].pose.position.y))
            c = np.array(
                (path.poses[i+2].pose.position.x, path.poses[i+2].pose.position.y))
            curvature[i] = menger_curvature(a, b, c)

    curvature[curvature < 1e-05] = 1e-05
    np.set_printoptions(precision=5)
    np.set_printoptions(suppress=True)
    np.set_printoptions(threshold=np.inf)
    # print(("curvature"))
    # print(curvature)

    speed_lim_curv = np.sqrt(lat_acc_max / curvature)
    speed_signal = np.minimum(speed_lim_curv, speed_limit)
    speed_limit_pts = speed_signal.tolist()

    # print("before filter")
    # print(speed_signal)

    if speed_signal.tolist().count(speed_signal[0]) is not len(speed_signal):
        b, a = butter(10, 0.125)
        speed_signal_filter = filtfilt(b, a, speed_signal)
        np.clip(speed_signal_filter, 0.0, speed_limit, speed_signal_filter)
        # print("after filter")
        # print(speed_signal_filter)
        speed_limit_pts = speed_signal_filter.tolist()

    return speed_limit_pts


def menger_curvature(a, b, c,):
    # Formula curvature
    twice_triangle_area = (b[0] - a[0])*(c[1] - a[1]) - \
        (b[1]-a[1]) * (c[0]-a[0])
    curvature = (2 * twice_triangle_area /
                 (np.linalg.norm(a - b) * np.linalg.norm(b - c) * np.linalg.norm(c - a)))

    return abs(curvature)


def cleanup_close_points(cl_path: Path) -> Path:

    min_dist = 1e-03

    skip_previous = False
    p_prev = Pose2D()
    new_path = Path()
    for i in range(len(cl_path.poses)):
        if not skip_previous:
            p_prev = cl_path.poses[i]
            p_prev_aux = np.array(
                (p_prev.pose.position.x, p_prev.pose.position.y))
            new_path.poses.append(p_prev)

        if i == len(cl_path.poses) - 1:
            break

        # Make sure that there are no repeated points
        p_next = np.array(
            (cl_path.poses[i+1].pose.position.x, cl_path.poses[i+1].pose.position.y))
        init_dist = np.linalg.norm(p_prev_aux - p_next)
        if (init_dist < min_dist):
            skip_previous = True
        else:
            skip_previous = False

    return new_path