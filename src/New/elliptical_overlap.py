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

def intersections(a, b):
    ea = LinearRing(a)
    eb = LinearRing(b)

    print(Polygon(ea).area)
    print(Polygon(eb).area)
    mp = ea.intersection(eb)

    pts = []
    for p in mp.geoms:
        pts.append([p.x, p.y])

    poly = Polygon(pts)
    print(poly.area)
    x = [p.x for p in mp.geoms]
    y = [p.y for p in mp.geoms]
    return x, y

ellipses = [(1, 1, 2, 1, 45), (2, 0.5, 5, 1.5, -30)]
a, b = ellipse_polyline(ellipses)
x, y = intersections(a, b)
plt.plot(x, y, "o")
plt.plot(a[:,0], a[:,1])
plt.plot(b[:,0], b[:,1])

plt.show()
