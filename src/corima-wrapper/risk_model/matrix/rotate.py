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

from numbers import Number

import numpy as np
from numpy.typing import NDArray

from .calc import front_rear_multiply_matrix_and_matrix_2d


def _rotation_matrix_2d(angle: Number) -> NDArray[np.float64]:
    """
    Create a 2D rotation matrix from a single radian value

    Args:
        angle: Single value in radians

    Returns:
        Single 2D matrix
    """

    cos = np.cos(angle)  # type:ignore[call-overload]
    sin = np.sin(angle)  # type:ignore[call-overload]

    rot = np.array([[cos, -sin], [sin, cos]], copy=False, dtype=np.float64)

    return rot


def _rotation_matrices_2d(angle: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Create a 2D rotation matrix or an array of rotation matrices

    Args:
        angle: numpy array in radians

    Returns:
        Single 2D matrix or array of 2D matrices
    """
    length = len(angle)
    cos = np.cos(angle).reshape((length, 1))
    sin = np.sin(angle).reshape((length, 1))
    rot: NDArray[np.float64] = np.concatenate([cos, -sin, sin, cos], axis=1).reshape((length, 2, 2))
    return rot


def rotate_matrix_2d(matrix: NDArray[np.float64], angle: Number | NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Rotate a 2D matrix by a certain angle. Also, applicable for an array of matrices and an array of angles.
    If <rot> is the rotation matrix for <angle>, it calculates rot . <matrix> . <rot>^T. This function
    is e.g. used for calculating the variance matrices of the 2D oriented Gaussian functions.

    Args:
        matrix: 2D matrix or array of 2D matrices
        angle: Angle or array of angles

    Returns:
        The rotated 2D matrix or array of 2D matrices
    """

    # rotation is expensive, so return matrix if angles are zero

    # Create rotation matrix from angle
    if isinstance(angle, Number):
        rot = _rotation_matrix_2d(angle)
    else:
        # will return unmodified matrix if the angles are all 0.0
        #
        # for else docu:
        # https://docs.python.org/3/tutorial/controlflow.html?highlight=else#break-and-continue-statements-and-else-clauses-on-loops
        for ang in angle:
            if ang != 0.0:
                break
        else:
            return matrix

        rot = _rotation_matrices_2d(angle)
    # Rotate covariance matrix calculating <rot> . <matrix> . <rot>^T
    return front_rear_multiply_matrix_and_matrix_2d(matrix, rot)
