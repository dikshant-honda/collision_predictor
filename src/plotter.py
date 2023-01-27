import matplotlib.pyplot as plt

file = open("traj_car_1.txt", "r+")

lines = file.read().split(',')

x, y = [], []
for line in lines:  
    print(line)  
    points = line.split()

for i in range(0, len(points), 2):
    x.append(float(points[i]))
    y.append(float(points[i+1]))

plt.plot(x,y)
plt.show()