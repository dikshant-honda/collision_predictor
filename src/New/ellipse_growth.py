import numpy as np
import matplotlib.pyplot as plt

center = [0, 0]

a = 4
b = 2

theta = np.linspace(0, 2 * np.pi, 100)
x = center[0] + a * np.cos(theta)
y = center[1] + b * np.sin(theta)

# plt.plot(x, y)

# plt.axis("equal")
# plt.show()

scale_factor  = 1.4

v = 2
for t in range(12):
    center[0] += v * t

    x = scale_factor * (center[0] + a * np.cos(theta))
    y = scale_factor * (center[1] + b * np.sin(theta)) 

    plt.plot(x, y)
    plt.draw()
    plt.axis('equal')
    plt.pause(0.7)

plt.show()