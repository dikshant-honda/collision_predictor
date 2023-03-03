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


def simple_inverse(matrix: NDArray[np.float64]) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    vectorized matrix inversion for 2x2 matrices
    see https://www.mathcentre.ac.uk/resources/uploaded/sigma-matrices7-2009-1.pdf
    difference to np.linalg.inv() can get quite big (1e-7) if matrix entries are large (1000+) due to floating point
    discretization

    Returns:
        Tuple of two matrices, first is the inverse, second is the determinant
    """
    assert (
        matrix.shape[-1] == 2 and matrix.shape[-2] == 2 and matrix.ndim in [2, 3]
    ), f"Matrix {matrix} does not match shape of (2,2) (or (:,2,2)), but has shape of {matrix.shape}"

    # expand dimensionality if only a single matrix is given
    is_single_matrix = matrix.ndim < 3
    if is_single_matrix:
        matrix = np.expand_dims(matrix, axis=0)
    # apply formula
    determinant = matrix[:, 0, 0] * matrix[:, 1, 1] - matrix[:, 0, 1] * matrix[:, 1, 0]
    inverse_matrix = (1 / determinant) * np.array(
        [[matrix[:, 1, 1], -matrix[:, 0, 1]], [-matrix[:, 1, 0], matrix[:, 0, 0]]], copy=False, dtype=np.float64
    )
    # reshape result and swap axes to match expected output format
    inverse_matrix = inverse_matrix.reshape(2, 2, matrix.shape[0]).swapaxes(1, 2).swapaxes(0, 1)
    # get rid of first axis if only single matrix shall be returned
    if is_single_matrix:
        return np.squeeze(inverse_matrix), np.squeeze(determinant)

    return inverse_matrix, determinant
