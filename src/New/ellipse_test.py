from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.affinity import rotate
from shapely.affinity import scale

import matplotlib.pyplot as plt

def ellipse(center_x, center_y, major_axis, minor_axis, rotation_angle=0):
    # Create an ellipse geometry using a scaled circle
    circle = Point(center_x, center_y).buffer(1)
    ellipse = scale(circle, major_axis, minor_axis)
    
    # Rotate the ellipse if required
    if rotation_angle != 0:
        ellipse = rotate(ellipse, rotation_angle, origin=(center_x, center_y))
    
    return ellipse

def overlap_area(ellipse1, ellipse2):
    # Find the intersection of the two ellipses
    intersection = ellipse1.intersection(ellipse2)
    
    # Calculate the area of the intersection
    area = intersection.area
    
    return area

# Example usage
ellipse1 = ellipse(center_x=0, center_y=0, major_axis=5, minor_axis=3, rotation_angle=45)
ellipse2 = ellipse(center_x=2, center_y=2, major_axis=4, minor_axis=2)

# plt.plot(ellipse1[:, 0], ellipse1[:, 1])

overlap_area = overlap_area(ellipse1, ellipse2)
print("Overlap Area:", overlap_area)
