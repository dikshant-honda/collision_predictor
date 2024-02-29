# -*- coding: utf-8 -*-
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

import numpy as np
from numpy.typing import NDArray

from risk_model.model import UncertainTrajectory

from .gauss import gauss_2d


def calculate_overlaps(
    source_trajectory: UncertainTrajectory, target_trajectories: list[UncertainTrajectory]
) -> NDArray[np.float64]:
    """Calculate the overlap between one and multiple other trajectories."""

    overlaps = [
        gauss_2d(
            source_trajectory.positions,
            target_trajectory.positions,
            source_trajectory.covariance_matrices + target_trajectory.covariance_matrices,
        )
        for target_trajectory in target_trajectories
    ]
    return np.minimum(overlaps, 1.0, dtype=np.float64)
