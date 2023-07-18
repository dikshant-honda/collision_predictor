import numpy as np
import matplotlib.pyplot as plt

# fig, ax = plt.subplots()
# ax.set_aspect('equal')

# velocity = 3
# radius = 2.5
# position = [0, 0]
# growth_rate = 1

# theta = np.linspace(0, 2*np.pi, 100)

# for time in range(10):
#     position[0] = velocity * time

#     if growth_rate < 1.2 :
#         growth_rate += 0.14 * time

#     x =  position[0] + growth_rate * radius * np.cos(theta)
#     y =  position[1] + growth_rate * radius * np.sin(theta)

#     ax.plot(x, y)
#     ax.plot(position[0], position[1], "*")

#     plt.pause(1)
#     # ax.clear()
# plt.show()


def noise(position, velocity, radius, time_horizon, growth_rate = 1):
    theta = np.linspace(0, 2*np.pi, 100)

    for time in range(time_horizon):
        position[0] = velocity * time       # for time being consider movement only in x direction, change later

        if growth_rate < 1.4:
            growth_rate += velocity * (time/50)

        x =  position[0] + growth_rate * radius * np.cos(theta)
        y =  position[1] + growth_rate * radius * np.sin(theta)

        ax.plot(x, y)
        ax.plot(position[0], position[1], "*") 

        plt.pause(0.5)

if __name__ == "__main__":
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    noise([0, 0], 3, 2.5, 10, 1)

    plt.show()