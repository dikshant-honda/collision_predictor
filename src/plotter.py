#! /usr/bin/env python3

def plotter(car):
    file_name = "traj_"+car+".txt"
    file = open(file_name, "r+")

    lines = file.read().split(',')

    x, y = [], []

    for line in lines: 
        points = line.split()

    for i in range(0, len(points), 2):
        x.append(float(points[i]))
        y.append(float(points[i+1]))

    return x, y