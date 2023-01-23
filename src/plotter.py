import matplotlib.pyplot as plt

file = open("/home/dikshant/catkin_ws/future_waypoints_car_3.txt", "r+")

lines = file.read().split(',')

x, y = [], []
for line in lines:    
    pass

plt.plot(x,y)
plt.show()