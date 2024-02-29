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

import math

import numpy as np
from numpy.typing import NDArray

from risk_model.matrix import front_rear_multiply_matrix_and_vector_2d, simple_inverse

EPSILON = np.finfo(float).eps


def gauss_2d(
    mean_1: NDArray[np.float64],
    mean_2: NDArray[np.float64],
    sigma2: NDArray[np.float64],
) -> NDArray[np.float64]:
    """
    Calculate multiplication and integration of two 2D Gaussians

    Args:
        mean_1: Center of first gaussian(s): (e.g. [x,y]) or array of (e.g. [[x1,y1], [x2,y2], ...])
        mean_2: Center of second gaussian(s): (e.g. [x,y]) or array of (e.g. [[x1,y1], [x2,y2], ...])
        sigma2: Gaussian variance matrix (generalizatopn of 1D squared variance, e.g. of form [[vx^2, 0], [0, vy^2]]) or
                array of, already rotated
    """
    # Ensure variance matrices with valid sigmas. Enabling for calculation of inverse.
    # Due to performance each variance matrix is changed independent of actual sigma values.
    sigma2 += np.array(((EPSILON, 0.0), (0.0, EPSILON)))
    mean = mean_1 - mean_2
    sigma2_inv, sigma2_det = simple_inverse(sigma2)

    # Matrix multiply each variance matrix by each vector: arg^2 = p_xy^T . sigma2^{-1} . p_xy
    arg = front_rear_multiply_matrix_and_vector_2d(sigma2_inv, mean)

    # Prepare return values

    result = np.exp(-arg / 2.0), np.sqrt(math.tau * sigma2_det)

    return (result[1][0] / result[1]) * result[0]  # type: ignore[no-any-return]
