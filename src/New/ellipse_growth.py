import numpy as np
import matplotlib.pyplot as plt

center = (0, 0)
init_raidus = 1
growth_rate = 0.1
num_points = 100

time = np.linspace(0, 1, num_points)

radii = init_raidus + growth_rate * time

theta = np.linspace(0, 2 * np.pi, num_points)
x = center[0] + radii * np.cos(theta)
y = center[1] + radii * np.sin(theta)

plt.plot(x, y)
plt.axis('equal')
plt.show()