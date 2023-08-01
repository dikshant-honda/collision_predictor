#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from shapely import MultiPoint, Polygon
from shapely.geometry.polygon import LinearRing
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.affinity import rotate
from shapely.affinity import scale


def ellipse(
        center_x: float, 
        center_y: float, 
        major_axis: float, 
        minor_axis: float, 
        rotation_angle: float = 0,
        ):
    """
    function to create an ellipse geometry using a scaled circle

    args:
        center_x: x coordinate of the origin of the ellipse
        center_y: y coordinate of the origin of the ellipse
        major_axis: length of major axis of the ellipse
        minor_axis: length of minor axis of the ellipse
        rotation_angle: orientation of the ellipse
    """
    
    circle = Point(center_x, center_y).buffer(1)
    ellipse = scale(circle, major_axis, minor_axis)
 
    if rotation_angle != 0:
        ellipse = rotate(ellipse, rotation_angle, origin=(center_x, center_y))
    
    return ellipse

def overlap_area(
        ellipse1: ellipse, 
        ellipse2: ellipse,
        ):
    """
    function to compute the overlap area of two ellipses

    args:
        ellipse1: uncertainity in the ego vehicle position
        ellipse2: uncertainity in the traffic vehicle position
    """

    # check if it intersects
    if ellipse1.intersects(ellipse2):
        intersect = ellipse1.intersection(ellipse2)
        area = intersect.area

        return area
    
    return 0

def plotter(
        ax,
        vehicle_1_data: list,
        vehicle_2_data: list,
) -> None:
    """
    Function to visualize the elliptical overlap dynamically at every time step

    args:
        ax: plotting tool
        vehicle_1_data: future trajectory information with uncertainity size for vehicle 1, 
        vehicle_2_data: future trajectory information with uncertainity size for vehicle 2
    """

    # ax.clear()
    vehicle_1_centers = vehicle_1_data[0]
    vehicle_1_size = vehicle_1_data[1]

    vehicle_2_centers = vehicle_2_data[0]
    vehicle_2_size = vehicle_2_data[1]

    vehicle_1_params = (vehicle_1_centers.x, vehicle_1_centers.y,
                        vehicle_1_size[0], vehicle_1_size[1], vehicle_1_size[2])
    vehicle_2_params = (vehicle_2_centers.x, vehicle_2_centers.y,
                        vehicle_2_size[0], vehicle_2_size[1], vehicle_2_size[2])

    vehicle_1, vehicle_2 = ellipse_polyline(
        [vehicle_1_params, vehicle_2_params])

    # visualization parameters
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_xlim(-1, 200)
    ax.set_ylim(-1.4, 1.4)

    ax.plot(vehicle_1[:, 0], vehicle_1[:, 1], 'r')
    ax.plot(vehicle_2[:, 0], vehicle_2[:, 1], 'b')

    plt.draw()
    plt.pause(0.1)

    return


def collision_probability(
        overlap: float,
        total: float,
) -> float:
    """
    dice loss of the overlap
    """
    return overlap / total


def overlap(
        vehicle_1_data: list,
        vehicle_2_data: list,
) -> float:
    """
    Function to compute the normalized overlap area of two interescting circles
    at every prediction time step

    args:
        vehicle_1_data: future trajectory information with uncertainity size for vehicle 1, 
        vehicle_2_data: future trajectory information with uncertainity size for vehicle 2
    """
    vehicle_1_centers = vehicle_1_data[0]
    vehicle_1_size = vehicle_1_data[1]

    vehicle_2_centers = vehicle_2_data[0]
    vehicle_2_size = vehicle_2_data[1]

    print(vehicle_1_centers.x, vehicle_1_centers.y, vehicle_1_size)
    print(vehicle_2_centers.x, vehicle_2_centers.y, vehicle_2_size)

    vehicle_1_params = (vehicle_1_centers.x, vehicle_1_centers.y,
                        vehicle_1_size[0], vehicle_1_size[1], vehicle_1_size[2])
    vehicle_2_params = (vehicle_2_centers.x, vehicle_2_centers.y,
                        vehicle_2_size[0], vehicle_2_size[1], vehicle_2_size[2])

    vehicle_1, vehicle_2 = ellipse_polyline(
        [vehicle_1_params, vehicle_2_params])

    intersect = intersection_points(vehicle_1, vehicle_2)

    overlap_ = overlap_area(intersect)

    vehicle_1_variance = ellipse_area(vehicle_1)
    vehicle_2_variance = ellipse_area(vehicle_2)

    total_variance = vehicle_1_variance + vehicle_2_variance

    probability = collision_probability(overlap_, total_variance)

    return probability

def plot(ellipse_1, ellipse_2):
    """
    plotting tool
    """
    plt.plot(ellipse_1[:, 0], ellipse_1[:, 1])
    plt.plot(ellipse_2[:, 0], ellipse_2[:, 1])

    return

# """
# testing code:


if __name__ == "__main__":
    # plotting tools
    fig, ax = plt.subplots()
    ax.axis('equal')

    ellipse_1_params = (1, 1, 6, 2, 0)
    ellipse_2_params = (2, 0, 5, 1.5, 0)

    ellipse_1_params = (1, 1, 2, 1, 45)
    ellipse_2_params = (2, 0.5, 5, 1.5, -30)

    ellipse_1, ellipse_2 = ellipse_polyline(
        [ellipse_1_params, ellipse_2_params])

    plot(ellipse_1, ellipse_2)
    plt.show()
    intersect = intersection_points(ellipse_1, ellipse_2)

    overlap = overlap_area(intersect)

    ellipse_1_area = ellipse_area(ellipse_1)
    ellipse_2_area = ellipse_area(ellipse_2)

    total_area = ellipse_1_area + ellipse_2_area

    prob = collision_probability(overlap, total_area)

    print(prob)

# """
