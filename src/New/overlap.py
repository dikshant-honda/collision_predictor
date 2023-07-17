import random
import math

def point_in_ellipse(x, y, a, b):
    return (x**2) / (a**2) + (y**2) / (b**2) <= 1

def estimate_overlap_area(x1, y1, a1, b1, theta1, x2, y2, a2, b2, theta2, num_points=100000):
    num_overlap_points = 0

    for _ in range(num_points):
        # Generate a random point within the bounding rectangle of the first ellipse
        x = random.uniform(x1 - a1, x1 + a1)
        y = random.uniform(y1 - b1, y1 + b1)

        # Rotate the point back to the original coordinate system of the first ellipse
        x_rot = (x - x1) * math.cos(-theta1) - (y - y1) * math.sin(-theta1)
        y_rot = (x - x1) * math.sin(-theta1) + (y - y1) * math.cos(-theta1)

        # Translate the point by the center offset of the second ellipse
        x_trans = x_rot + (x2 - x1)
        y_trans = y_rot + (y2 - y1)

        # Check if the point is inside both ellipses
        if point_in_ellipse(x_rot, y_rot, a1, b1) and point_in_ellipse(x_trans, y_trans, a2, b2):
            num_overlap_points += 1

    # Estimate the overlap area based on the ratio of overlap points to total points
    area1 = math.pi * a1 * b1
    overlap_area = (num_overlap_points / num_points) * area1

    return overlap_area

# Ellipse 1 parameters
x1 = 0  # x-coordinate of ellipse 1 center
y1 = 0  # y-coordinate of ellipse 1 center
a1 = 5  # semi-major axis of ellipse 1
b1 = 3  # semi-minor axis of ellipse 1
theta1 = math.radians(0)  # rotation angle of ellipse 1 in radians

# Ellipse 2 parameters
x2 = 0  # x-coordinate of ellipse 2 center
y2 = 0  # y-coordinate of ellipse 2 center
a2 = 5  # semi-major axis of ellipse 2
b2 = 3  # semi-minor axis of ellipse 2
theta2 = math.radians(0)  # rotation angle of ellipse 2 in radians

# Estimate the overlap area
overlap_area = estimate_overlap_area(x1, y1, a1, b1, theta1, x2, y2, a2, b2, theta2)

# Print the estimated overlap area
print("Estimated Overlap Area:", overlap_area)
