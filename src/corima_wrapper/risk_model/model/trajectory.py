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

def rosPoint_to_shapelyPoint(points):
    ls = []
    for i in points:
        ls.append(Point(i.x, i.y, 0.0))
    return ls

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

@dataclass(frozen=True)
class UncertainTrajectory(Trajectory):
    covariance_matrices: NDArray[np.float64]
