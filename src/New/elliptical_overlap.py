#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
from shapely import MultiPoint, Polygon
from shapely.geometry.polygon import LinearRing


def ellipse_polyline(
        ellipses: list,
        n: int = 100,
) -> list:
    """
    Function to draw an ellipse from the ellipses params

    Return the list of elliptical coordinates

    args:
        ellipses: list of ellipse parameters -> [center.x, center.y, major_axis, minor_axis, orientation]
        n: number of points 
    """
    theta = np.linspace(0, 2*np.pi, n, endpoint=False)
    st = np.sin(theta)
    ct = np.cos(theta)
    result = []

    for x, y, a, b, angle in ellipses:
        angle = np.deg2rad(angle)
        sa = np.sin(angle)
        ca = np.cos(angle)
        p = np.empty((n, 2))
        p[:, 0] = x + a * ca * ct - b * sa * st
        p[:, 1] = y + a * sa * ct + b * ca * st
        result.append(p)

    return result


def overlap_area(
        intersection_points: MultiPoint,
) -> float:
    """
    Function to compute the area of the onverlapped trapezium

    args:
        intersection_points: Multipoint list of intersected points
    """
    points = []
    for point in intersection_points.geoms:
        points.append([point.x, point.y])

    return Polygon(points).area


def ellipse_area(
        ellipse_coordinates: list,
) -> float:
    """
    Function to compute the area of the ellipse

    args:
        ellipse_coordinates: coordinates of the ellipse in consideration
    """
    return Polygon(LinearRing(ellipse_coordinates)).area


def intersection_points(
        ellipse_1: list,
        ellipse_2: list,
) -> MultiPoint:
    """
    Function to find out the intersection points of two ellipses

    args:
        ellipse_1: coordinates of ellipse 1
        ellipse_2: coordinates of ellipse 2
    """
    e1 = LinearRing(ellipse_1)
    e2 = LinearRing(ellipse_2)

    return e1.intersection(e2)


def multipoint_to_list(
        points: MultiPoint,
) -> list:
    """
    Function to convert the multipoints into list
    """
    points = []
    for point in intersection_points.geoms:
        points.append([point.x, point.y])

    return points


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

    vehicle_1_params = (vehicle_1_centers.x, vehicle_1_centers.y, vehicle_1_size[0], vehicle_1_size[1], vehicle_1_size[2])
    vehicle_2_params = (vehicle_2_centers.x, vehicle_2_centers.y, vehicle_2_size[0], vehicle_2_size[1], vehicle_2_size[2])

    vehicle_1, vehicle_2 = ellipse_polyline([vehicle_1_params, vehicle_2_params])


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

    vehicle_1_params = (vehicle_1_centers.x, vehicle_1_centers.y, vehicle_1_size[0], vehicle_1_size[1], vehicle_1_size[2])
    vehicle_2_params = (vehicle_2_centers.x, vehicle_2_centers.y, vehicle_2_size[0], vehicle_2_size[1], vehicle_2_size[2])

    vehicle_1, vehicle_2 = ellipse_polyline([vehicle_1_params, vehicle_2_params])

    intersect = intersection_points(vehicle_1, vehicle_2)

    overlap_ = overlap_area(intersect)

    vehicle_1_variance = ellipse_area(vehicle_1)
    vehicle_2_variance = ellipse_area(vehicle_2)

    total_variance = vehicle_1_variance + vehicle_2_variance

    probability = collision_probability(overlap_, total_variance)

    return probability


# """
# testing code:


if __name__ == "__main__":
    # plotting tools
    fig, ax = plt.subplots()
    ax.axis('equal')

    ellipse_1_params = (1, 1, 2, 1, 45)
    ellipse_2_params = (2, 0.5, 5, 1.5, -30)

    ellipse_1, ellipse_2 = ellipse_polyline(
        [ellipse_1_params, ellipse_2_params])

    plotter(ax, ellipse_1, ellipse_2)

    intersect = intersection_points(ellipse_1, ellipse_2)

    overlap = overlap_area(intersect)

    ellipse_1_area = ellipse_area(ellipse_1)
    ellipse_2_area = ellipse_area(ellipse_2)

    total_area = ellipse_1_area + ellipse_2_area

    prob = collision_probability(overlap, total_area)

    print(prob)

    plt.show()

# """
