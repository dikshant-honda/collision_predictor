#! /usr/bin/env python3

from IDM.frenet import Point2D


class Path:
    def __init__(self, x: list, y: list) -> None:
        self.x = x
        self.y = y
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

    def get_path(self):
        path = self.coordinates_to_points(self.x, self.y)

        return path
