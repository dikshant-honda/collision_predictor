#! /usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from IDM.frenet import Point2D


class Path:
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.number_of_points = 100

    def points_to_coordinates(
            self,
            points: list,
    ):
        """
        Function to convert 2D points to list of coordinates

        args:
            ponts: list of points stored in point2D format
        """
        x_coordinates, y_coordinates = [], []

        for point in points:
            x_coordinates.append(point.x)
            y_coordinates.append(point.y)

        return x_coordinates, y_coordinates

    def coordinates_to_points(self, x, y):
        path = []
        for i in range(self.number_of_points):
            path.append(Point2D(x[i], y[i]))

        return path

    def path(self):
        self.x = np.linspace(0, 100, self.number_of_points)
        self.y = np.linspace(0, 0, self.number_of_points)

        return self.x, self.y
    
    def get_path(self):
        self.path()
        path = self.coordinates_to_points(self.x, self.y)

        return path


pt = Path()
path = pt.get_path()

