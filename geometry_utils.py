import numpy as np

class Point2D:
    def __init__(self,x_init,y_init):
        self.x = x_init
        self.y = y_init

    def shift(self, x, y):
        self.x += x
        self.y += y

    def __repr__(self):
        return "".join(["Point(", str(self.x), ",", str(self.y), ")"])

class Point3D(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return "(%i, %i, %i)"%(self.x,self.y,self.z)

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