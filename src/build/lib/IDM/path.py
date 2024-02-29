#! /usr/bin/env python3

from IDM.frenet import Point2D


class Path:
    def __init__(
            self,
            x: list,
            y: list,
            number_of_points: int,
    ) -> None:
        self.x = x
        self.y = y
        self.number_of_points = number_of_points

    def points_to_coordinates(
            self,
            points: list,
    ) -> list:
        """
        Function to convert 2D points to list of coordinates

        args:
            points: list of points stored in point2D format
        """
        x_coordinates, y_coordinates = [], []

        for point in points:
            x_coordinates.append(point.x)
            y_coordinates.append(point.y)

        return x_coordinates, y_coordinates

    def coordinates_to_points(
        self,
        x: list,
        y: list,
    ) -> list:
        """
        Function to convert the separates x,y coordinates list to points format
        """
        path = []
        for i in range(self.number_of_points):
            path.append(Point2D(x[i], y[i]))

        return path

    def get_path(
        self
    ) -> list:
        """
        Function just to get the finally Point2D path list
        """
        path = self.coordinates_to_points(self.x, self.y)

        return path
