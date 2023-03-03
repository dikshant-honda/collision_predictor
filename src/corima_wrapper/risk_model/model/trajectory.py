# -*- coding: utf-8 -*-
#
#
# Copyright (C)
# Honda Research Institute Europe GmbH
# Carl-Legien-Str. 30
# 63073 Offenbach/Main
# Germany
#
# UNPUBLISHED PROPRIETARY MATERIAL.
# ALL RIGHTS RESERVED.
#
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from shapely import Point, LineString

from risk_model.matrix import rotate_matrix_2d

from .types import Position
from .uncertainty_config import UncertaintyConfig
from .velocity import Velocity

import dubins

# def get_dubins(start, end, theta0, theta1, step_size = 0.5):
# 	q0 = (start[0], start[1], theta0)
# 	q1 = (end[0], end[1], theta1)

# 	turning_radius = 1.55
# 	path = dubins.shortest_path(q0, q1, turning_radius)
	
# 	configurations, _ = path.sample_many(step_size)

# 	x, y, yaw = np.array([]), np.array([]), np.array([])
# 	for i in range(len(configurations)):
# 		x = np.append(x, configurations[i][0])
# 		y = np.append(y, configurations[i][1])
# 		if np.pi <= configurations[i][2] <= 2*np.pi:
# 			yaw = np.append(yaw, 2*np.pi-configurations[i][2])
# 		else:
# 			yaw = np.append(yaw, configurations[i][2])
	
# 	return x, y, yaw

# def get_straight_dubins(start, end, theta0, theta1, step_size = 0.1):
# 	q0 = (start[0], start[1], theta0)
# 	q1 = (end[0], end[1], theta1)

# 	turning_radius = 0.0001

# 	path = dubins.shortest_path(q0, q1, turning_radius)
# 	configurations, _ = path.sample_many(step_size)

# 	x, y, yaw = np.array([]), np.array([]), np.array([])
# 	for i in range(len(configurations)):
# 		x = np.append(x, configurations[i][0])
# 		y = np.append(y, configurations[i][1])
# 		if np.pi <= configurations[i][2] <= 2*np.pi:
# 			yaw = np.append(yaw, 2*np.pi-configurations[i][2])
# 		else:
# 			yaw = np.append(yaw, configurations[i][2])

# 	return x, y, yaw

# def arr_to_point(x, y):
# 	point_arr = []
# 	for i in range(len(x)):
# 		point_arr.append(Point(x[i], y[i], 0))
# 	return point_arr

def rosPoint_to_shapelyPoint(points):
    ls = []
    for i in points:
        ls.append(Point(i.x, i.y, 0.0))
    return ls

# def distance(x1, y1, x2, y2):
#     return np.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))

# def closest_pt_idx(x, y, lane):
#     # finding the closest index on lane from point(x,y)
#     closest_index = 0
#     min_dist = 10000.0
#     for i in range(len(lane)):
#         dist = distance(lane[i].x, lane[i].y, x, y)
#         if dist < min_dist:
#             min_dist = dist
#             closest_index = i
#     return closest_index

@dataclass(frozen=True)
class Trajectory:
    id: str
    delta_t: float
    positions: NDArray[np.float64]

    _position_deltas: NDArray[np.float64] = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "_position_deltas", np.diff(self.positions, axis=0))

    def angles(self) -> NDArray[np.float64]:
        """List of angles between {self.positions}."""
        angles_from_deltas = np.arctan2(self._position_deltas[:, 1], self._position_deltas[:, 0])
        return np.append(angles_from_deltas, angles_from_deltas[-1])

    def traveled_distances(self) -> NDArray[np.float64]:
        """List of distances between {self.positions}."""
        distances_from_positions = np.linalg.norm(self._position_deltas, axis=1)
        return np.append(distances_from_positions, distances_from_positions[-1])
    
    def uncertain(
        self,
        uncertainty_config: UncertaintyConfig = UncertaintyConfig(),
    ) -> UncertainTrajectory:
        """
        Function to add 'uncertainty' to a {Trajectory}.

        By default we use our default {UncertaintyConfig} which values are based on a Pedestrian.
        """
        traveled_distances = self.traveled_distances()

        sigma_max = uncertainty_config.calc_speed_dependant_max_sigma(speed=max(traveled_distances) / self.delta_t)

        factors_x = traveled_distances * uncertainty_config.speed_factor[0]
        factors_y = traveled_distances * uncertainty_config.speed_factor[1]

        factors_x[0] = uncertainty_config.sigma_start.x
        factors_y[0] = uncertainty_config.sigma_start.y

        sigmas_x = np.square(np.minimum(np.cumsum(factors_x), sigma_max.x))
        sigmas_y = np.square(np.minimum(np.cumsum(factors_y), sigma_max.y))

        zeros = np.zeros(traveled_distances.size)

        covariance_matrices: NDArray[np.float64] = (
            np.array(  # pylint: disable=too-many-function-args
                [sigmas_x, zeros, zeros, sigmas_y], dtype=np.float64, copy=False
            )
            .transpose()
            .reshape(traveled_distances.size, 2, 2)
        )

        covariance_matrices = rotate_matrix_2d(covariance_matrices, self.angles())

        return UncertainTrajectory(
            id=self.id,
            delta_t=self.delta_t,
            positions=self.positions,
            covariance_matrices=covariance_matrices,
        )

    @classmethod
    def future_prediction(
        cls,
        trajectory_id: str,
        position: Position,
        velocity: Velocity,
        route: list,
        delta_t: float,
        trajectory_length: int,
    ) -> Trajectory:
        """Creates a future prediction for {delta_t * trajectory_length} seconds."""

        if velocity.speed == 0:
            positions = np.full((trajectory_length, 2), position, dtype=np.float64)
        else:
            route = rosPoint_to_shapelyPoint(route)
            line = LineString(route)
            pts = []
            for t in range(240):
                pts.append(velocity.speed*t*delta_t)

            traj = []

            for i in range(len(pts)):
                traj.append(line.interpolate(pts[i]))

            positions_x = [point.x for point in traj]
            positions_y = [point.y for point in traj]

            positions = (
                np.array([positions_x, positions_y], copy=False, dtype=np.float64)
                .transpose()
                .reshape(trajectory_length, 2)
            )

        return cls(
            id=trajectory_id,
            delta_t=delta_t,
            positions=positions,
        )

    @classmethod
    def linear_prediction(
        cls,
        trajectory_id: str,
        position: Position,
        velocity: Velocity,
        delta_t: float,
        trajectory_length: int,
    ) -> Trajectory:
        """Creates a linear prediction into the future for {delta_t * trajectory_length} seconds."""

        if velocity.speed == 0:
            positions = np.full((trajectory_length, 2), position, dtype=np.float64)
        else:
            static_positions_x = np.full(trajectory_length, position.x, dtype=np.float64)
            static_positions_y = np.full(trajectory_length, position.y, dtype=np.float64)

            static_distances_x = np.full(trajectory_length, velocity.x * delta_t, dtype=np.float64)
            static_distances_y = np.full(trajectory_length, velocity.y * delta_t, dtype=np.float64)

            static_distances_x[0] = 0
            static_distances_y[0] = 0

            positions_x = static_positions_x + np.cumsum(static_distances_x)
            positions_y = static_positions_y + np.cumsum(static_distances_y)

            positions = (
                np.array([positions_x, positions_y], copy=False, dtype=np.float64)
                .transpose()
                .reshape(trajectory_length, 2)
            )

        return cls(
            id=trajectory_id,
            delta_t=delta_t,
            positions=positions,
        )

    # @classmethod
    # def static_ego(cls, delta_t: float, trajectory_length: int) -> Trajectory:
    #     """Helper function for a static 'trajectory' at the Position(x=0, y=0)."""

    #     return cls.linear_prediction(
    #         "ego",
    #         Position(0, 0),
    #         Velocity(0, 0),
    #         delta_t=delta_t,
    #         trajectory_length=trajectory_length,
    #     )

# x1, y1, _ = get_straight_dubins([13, -0.9], [2.5, -0.9], np.pi, np.pi)
# x9, y9, _ = get_dubins([2.5, -0.9], [0.9, -2.5], np.pi, -np.pi/2)
# x6, y6, _ = get_straight_dubins([0.9, -2.5], [0.9, -13], -np.pi/2, -np.pi/2)

# x10, y10, _ = get_dubins([2.5, -0.9], [-2.5, -0.9], np.pi, np.pi)				
# x3, y3, yaw3 = get_straight_dubins([-2.5, -0.9], [-13, -0.9], np.pi, np.pi)   		
# x11, y11, yaw11 = get_dubins([2.5, -0.9], [-0.9, 2.5], np.pi, np.pi/2)				
# x7, y7, yaw7 = get_straight_dubins([-0.9, 2.5], [-0.9, 13], np.pi/2, np.pi/2) 		

# x5, y5, _ = get_straight_dubins([-0.9, -13], [-0.9, -2.5], np.pi/2, np.pi/2) 	
# x14, y14, _ = get_dubins([-0.9, -2.5], [2.5, 0.9], np.pi/2, 0)					
# x2, y2, _ = get_straight_dubins([2.5, 0.9], [13, 0.9], 0, 0)             		 

# x4, y4, _ = get_straight_dubins([-13, 0.9], [-2.5, 0.9], 0, 0)      	  			
# x15, y15, _ = get_dubins([-2.5, 0.9], [-0.9, 2.5], 0, np.pi/2)					

# x_1, y_1 = np.hstack((x5, x14, x2)), np.hstack((y5, y14, y2))
# x_2_left, y_2_left = np.hstack((x1, x9, x6)), np.hstack((y1, y9, y6))
# x_2_straight, y_2_straight = np.hstack((x1, x10, x3)), np.hstack((y1, y10, y3))
# x_2_right, y_2_right = np.hstack((x1, x11, x7)), np.hstack((y1, y11, y7))
# x_3, y_3 = np.hstack((x4, x15, x7)), np.hstack((y4, y15, y7))

# waypoints_1 = arr_to_point(x_1, y_1)
# waypoints_2_left = arr_to_point(x_2_left, y_2_left)
# waypoints_2_straight = arr_to_point(x_2_straight, y_2_straight)
# waypoints_2_right = arr_to_point(x_2_right, y_2_right)
# waypoints_3 = arr_to_point(x_3, y_3)

@dataclass(frozen=True)
class UncertainTrajectory(Trajectory):
    covariance_matrices: NDArray[np.float64]
