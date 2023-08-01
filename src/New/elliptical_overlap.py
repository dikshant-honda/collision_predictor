#! /usr/bin/env python3

import matplotlib.pyplot as plt
from shapely.affinity import rotate, scale
from shapely.geometry import Point


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

    vehicle_1 = ellipse(vehicle_1_centers.x, vehicle_1_centers.y,
                        vehicle_1_size[0], vehicle_1_size[1], vehicle_1_size[2])
    vehicle_2 = ellipse(vehicle_2_centers.x, vehicle_2_centers.y,
                        vehicle_2_size[0], vehicle_2_size[1], vehicle_2_size[2])

    vehicle_1_x, vehicle_1_y = coords_to_list(vehicle_1)
    vehicle_2_x, vehicle_2_y = coords_to_list(vehicle_2)

    # visualization parameters
    ax.set_xlabel("x(m)")
    ax.set_ylabel("y(m)")
    ax.set_xlim(-1, 200)
    ax.set_ylim(-1.4, 1.4)

    ax.plot(vehicle_1_x, vehicle_1_y, 'r')
    ax.plot(vehicle_2_x, vehicle_2_y, 'b')

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

    vehicle_1 = ellipse(vehicle_1_centers.x, vehicle_1_centers.y,
                        vehicle_1_size[0], vehicle_1_size[1], vehicle_1_size[2])
    vehicle_2 = ellipse(vehicle_2_centers.x, vehicle_2_centers.y,
                        vehicle_2_size[0], vehicle_2_size[1], vehicle_2_size[2])

    overlap_ = overlap_area(vehicle_1, vehicle_2)

    total_variance = vehicle_1.area + vehicle_2.area

    probability = collision_probability(overlap_, total_variance)

    return probability


def coords_to_list(ellipse_: ellipse):
    xx, yy = ellipse_.exterior.coords.xy
    return xx.tolist(), yy.tolist()


def plot(
        ellipse_1: ellipse,
        ellipse_2: ellipse
):
    """
    plotting tool
    """
    x_1, y_1 = coords_to_list(ellipse_1)
    x_2, y_2 = coords_to_list(ellipse_2)
    plt.plot(x_1, y_1, 'r-')
    plt.plot(x_2, y_2, 'b-')
    return

# """
# testing code:


if __name__ == "__main__":
    # plotting tools
    fig, ax = plt.subplots()
    ax.axis('equal')

    ellipse1 = ellipse(1, 1, 6, 2, -10)
    ellipse2 = ellipse(2, 1, 5, 1.5, 30)

    overlap = overlap_area(ellipse1, ellipse2)

    total_area = ellipse1.area + ellipse2.area

    collision = collision_probability(overlap, total_area)

    print(collision)

    plot(ellipse1, ellipse2)
    plt.show()

# """
