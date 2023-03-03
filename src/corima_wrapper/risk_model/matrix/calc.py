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
import numpy as np
from numpy.typing import NDArray


def front_rear_multiply_matrix_and_matrix_2d(
    middle_matrix: NDArray[np.float64], front_rear_matrix: NDArray[np.float64]
) -> NDArray[np.float64]:
    """
    Multiply 2D <middle_matrix> with a <front_rear_matrix> at the front (transposed) and at the back.
    Also applicable for an array of middle matrices and an array of front/rear matrices.
    It calculates <front_rear_matrix>^T . <middle_matrix> . <front_rear_matrix>.

    For further information on einsum and the underlying matrix operations have a look at:
    risk_model/doc/einstein_notation/einstein_notation.md

    Args:
        middle_matrix: 2D matrix or array of 2D matrices
        front_rear_matrix: 2D matrix or array of 2D matrices

    Returns:
        The resulting 2D matrix or array of 2D matrices
    """
    dim_middle_matrix = middle_matrix.ndim
    dim_front_rear_matrix = front_rear_matrix.ndim

    assert dim_middle_matrix in [2, 3] and dim_front_rear_matrix in [2, 3], (
        f"wrong combination of input dimensions "
        f"dim(middle_matrix)={dim_middle_matrix} "
        f"dim(front_rear_matrix)={dim_front_rear_matrix} "
        f"expected: "
        f"dim(middle_matrix)=2 or 3 "
        f"dim(front_rear_matrix)=2"
    )

    if dim_front_rear_matrix == 2 and dim_middle_matrix == 2:
        # Single middle matrix and single front/rear matrix
        ein = "jk,kl,ml->jm"
    elif dim_front_rear_matrix == 3 and dim_middle_matrix == 3:
        # Array of middle matrices and array of front/rear matrices
        ein = "ijk,ikl,iml->ijm"
    elif dim_front_rear_matrix == 2 and dim_middle_matrix == 3:
        # Single front/rear matrix and array of middle matrices
        ein = "jk,ikl,ml->ijm"
    else:  # dim_front_rear_matrix == 3 and dim_middle_matrix == 2:
        # Single middle matrix and array of front/rear matrices
        ein = "ijk,kl,iml->ijm"

    # ein = "jk,kl,ml->jm" if dim_middle_matrix == 2 else "jk,ikl,ml->ijm"
    # Calc <front_rear_matrix> . <middle_matrix> . <front_rear_matrix>^T
    return np.einsum(ein, front_rear_matrix, middle_matrix, front_rear_matrix)  # type: ignore[no-any-return]


def front_rear_multiply_matrix_and_vector_2d(
    middle_matrix: NDArray[np.float64], front_rear_vector: NDArray[np.float64]
) -> NDArray[np.float64]:
    """
    Multiply 2D <middle_matrix> with a <front_rear_vector> at the front (transposed) and at the back.
    Also, applicable for an array of middle matrices and an array of front/rear vectors.
    It calculates <front_rear_vector>^T . <middle_matrix> . <front_rear_vector>.

    For further information on einsum and the underlying matrix operations have a look at:
    risk_model/doc/einstein_notation/einstein_notation.md

    Args:
        middle_matrix: 2D matrix or array of 2D matrices
        front_rear_vector: 2D vector or array of 2D vectors

    Returns:
        The resulting value or an array of values
    """

    dim_fr_mat = front_rear_vector.ndim
    dim_m_mat = middle_matrix.ndim
    if dim_fr_mat == 1 and dim_m_mat == 2:
        # Single middle matrix and single front/rear vector
        ein = "j,jk,k"

    elif dim_fr_mat == 2 and dim_m_mat == 3:
        # Array of middle matrices and array of front/rear vectors
        ein = "ij,ijk,ik->i"

    elif dim_fr_mat == 1 and dim_m_mat == 3:
        # Single front/rear vector and array of middle matrices
        ein = "j,ijk,k->i"

    elif dim_fr_mat == 2 and dim_m_mat == 2:
        # Single middle matrix and array of front/rear vectors
        ein = "ij,jk,ik->i"

    else:
        assert False, "Inadmissible combination of input dimensions of <middle_matrix> and <front_rear_vector>"
    # Calc <front_rear_matrix>^T . <middle_matrix> . <front_rear_vector>
    return np.einsum(ein, front_rear_vector, middle_matrix, front_rear_vector)  # type: ignore[no-any-return]
