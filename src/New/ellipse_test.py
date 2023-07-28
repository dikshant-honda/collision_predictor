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
    # # Find the intersection of the two ellipses
    # intersection = ellipse1.intersection(ellipse2)

    # # Calculate the area of the intersection
    # area = intersection.area
    if ellipse1.intersects(ellipse2): 
        intersect = ellipse1.intersection(ellipse2).area
        return intersect
    return 0
    
    # return area

# Example usage
ellipse1 = ellipse(center_x=1, center_y=1, major_axis=6, minor_axis=2, rotation_angle=0)
ellipse2 = ellipse(center_x=15, center_y=0, major_axis=5, minor_axis=1.5)

xx, yy = ellipse1.exterior.coords.xy

x = xx.tolist()
y = yy.tolist()

plt.plot(x, y)

xx, yy = ellipse2.exterior.coords.xy

x = xx.tolist()
y = yy.tolist()

plt.plot(x, y)

overlap_area = overlap_area(ellipse1, ellipse2)
print("Overlap Area:", overlap_area)
