#! /usr/bin/env python3

import bisect
import math

import numpy as np
from numpy.typing import NDArray


class Point2D:
    def __init__(
        self,
        x_init: float = 0,
        y_init: float = 0,
    ) -> None:
        """
        class for defining the cartesian coordinates
        """
        self.x = x_init
        self.y = y_init


class Frenet:
    def __init__(
        self,
        s: float = 0,
        d: float = 0,
    ) -> None:
        """
        class for defining the frenet coordinates
        """
        self.s = s
        self.d = d


def global_to_local(
        reference_point: Point2D,
        orientation: float,
        p: Point2D,
) -> Point2D:
    """
    function for transforming global point to local point

    args:
        reference_point: closest reference point on the lane
        orientation: global orientation of the reference point on the lane
        p: point to be considered for the transformation
    """
    delta = Point2D(p.x - reference_point.x, p.y - reference_point.y)

    s = math.sin(-orientation)
    c = math.cos(-orientation)

    out = Point2D(delta.x * c - delta.y * s,
                  delta.x * s + delta.y * c)

    return out


def local_to_global(
        center: Point2D,
        theta: float,
        p: Point2D,
) -> Point2D:
    """
    function for transforming local point to global point

    args:
        center: closest reference point on the lane
        theta: local orientation
        point: point to be considered for transformation
    """
    s = math.sin(theta)
    c = math.cos(theta)

    out = Point2D(p.x * c - p.y * s + center.x, p.x * s + p.y * c + center.y)

    return out


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """
    function to compute the euclidean distance between two points

    args: 
        x1, y1: coordinates of point 1
        x2, y2: coordinates of point 2
    """
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)


def get_s_map(
    path: list,
) -> NDArray[np.float64]:
    """
    function to get the s-map in Frenet coordinate system.
    it will accumulate the distance along the curve taking origin as the vehicle current position.

    args: 
        path: 2D list having the coordinates of the middle lane of the road 
    """
    s_map = np.array([], dtype=np.float64)
    accumulated_distance = 0.0
    prev_point = Point2D()
    for point in path:
        if prev_point != Point2D(0, 0):
            accumulated_distance += distance(
                prev_point.x, prev_point.y, point.x, point.y)
        s_map = np.append(s_map, accumulated_distance)
        prev_point = point

    return s_map


def closest_point_ind(
        path: list,
        point: Point2D,
) -> int:
    """
    function to find the closest point index on the path from the given point

    args:
        path: 2D list having the coordinates of the middle lane of the road 
        point: point to be considered
    """
    closest_index = None
    min_dist = np.inf

    for idx, pt in enumerate(path):
        dist = distance(pt.x, pt.y, point.x, point.y)
        if dist < min_dist:
            min_dist = dist
            closest_index = idx

    return closest_index


def get_frenet(
        point: Point2D,
        path: list,
        s_map: NDArray[np.float64],
) -> Frenet:
    """
    function to convert cartesian coordinates system to frenet coordinate system

    args:
        x, y: point to be considered
        path: 2D list having the coordinates of the middle lane of the road 
        s_map: cumulative distance map from current point obtained using get_s_map() function
    """

    ind_closest = closest_point_ind(path, point)

    # vehicle is at infinity
    if ind_closest is None:
        return Frenet(np.inf, 0)

    # Determine the indices of the 2 closest points
    if ind_closest < len(path):
        # Check if we are at the end of the segment
        if ind_closest == len(path) - 1:
            use_previous = True
        elif ind_closest == 0:
            use_previous = False
        else:
            dist_prev = distance(
                path[ind_closest-1].x, path[ind_closest-1].y, point.x, point.y)
            dist_next = distance(
                path[ind_closest+1].x, path[ind_closest+1].y, point.x, point.y)

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
        local_p = global_to_local(p1, theta, point)

        # Get the coordinates in the Frenet frame
        p_s = s_map[prev_idx] + local_p.x
        p_d = local_p.y

    else:
        print("Incorrect index")
        return

    return Frenet(p_s, p_d)


def get_xy(
        point: Frenet,
        path: list,
        s_map: NDArray[np.float64],
) -> Point2D:
    """
    function to convert frenet coordinates system to cartesian coordinates system

    args:
        point: frenet coordinate system point to be considered
        path: 2D list having the coordinates of the middle lane of the road 
        s_map: cumulative distance map from current point obtained using get_s_map() function
    """

    # If the value is out of the actual path send a warning
    if point.s < 0.0 or point.s > s_map[-1]:
        if point.s < 0.0:
            prev_point = 0
        else:
            prev_point = len(s_map) - 2
    else:
        # Find the previous point
        idx = bisect.bisect_left(s_map, point.s)
        prev_point = idx - 1

    p1 = path[prev_point]
    p2 = path[prev_point + 1]

    # Transform from local to global
    theta = math.atan2(p2.y - p1.y, p2.x - p1.x)
    p_xy = local_to_global(p1, theta, Point2D(
        point.s - s_map[prev_point], point.d))

    return p_xy
