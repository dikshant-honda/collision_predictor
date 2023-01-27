#! /usr/bin/env python3

import os

def plotter(car):
    open_path = "/home/dikshant/catkin_ws/src/collision_predictor/src"
    file_name = "traj_"+car+".txt"
    file = open(os.path.join(open_path, file_name), "r+")

    lines = file.read().split(',')

    x, y = [], []

    for line in lines: 
        points = line.split()

    for i in range(0, len(points), 2):
        x.append(float(points[i]))
        y.append(float(points[i+1]))

    return x, y