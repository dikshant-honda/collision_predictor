from shapely import Point, LineString
import matplotlib.pyplot as plt

line = LineString([Point(-0.9, -10.0), Point(-0.9, -6.0), Point(-0.9, -2.0), Point(-0.1, -0.1), Point(2.0, 0.9), Point(6.0, 0.9), Point(10.0, 0.9)])

vel = 0.2
p = []

for t in range(100):
    p.append(vel*t)

trajectory = []

for i in range(len(p)):
    trajectory.append(line.interpolate(p[i]))

xs = [point.x for point in trajectory]
ys = [point.y for point in trajectory]

print(trajectory)
plt.scatter(xs, ys)
plt.show()
