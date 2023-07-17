import numpy as np
import matplotlib.pyplot as plt

center = [0, 0]

a = 4
b = 2

theta = np.linspace(0, 2 * np.pi, 100)

scale_factor  = 1.1

v = 2
for t in range(12):
    center[0] = v * t

    x = center[0] + (scale_factor * t * a) * np.cos(theta)
    y = center[1] + (scale_factor * t * b) * np.sin(theta)

    plt.plot(center[0], center[1], "*")
    plt.plot(x, y)
    plt.draw()
    plt.axis('equal')
    plt.pause(0.7)

plt.show()