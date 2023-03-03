# -*- coding: utf-8 -*-
#
# Tests for survival_probability()
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
#

from __future__ import annotations

from functools import reduce

import numpy as np
from numpy.ma import add
from numpy.typing import NDArray

from risk_model.model import Probability


def compute_survival(
    event_matrix: NDArray[np.float64],
    delta_t: float,
    tau_esc: float = 0.4,
) -> list[Probability]:
    """
    Calculate the survival function from the instantaneous event probabilities.

    1. Escape probability for this timestep
    2. Calculate the combined instantaneous event probabilities for all critical events (without escape probability)
    3. Calculate total instantaneous event probability by adding the escape rate
    4. Integrate to get the survival function
       Remark: The survival function is retarded by one step, because it has to start at 1.0!
       That is, the tiep[0] is valid from [0,delta_t[, but surv_func[0] is the survival probability
       until ]...,0]

    Args:
        event_matrix: A list of multiple lists of events, e.g. Collision Events
        delta_t: Duration between index steps
        tau_esc: Escape time / inverse of escape rate
    """
    normalized_event_matrix = np.minimum(
        reduce(add, event_matrix) + delta_t / tau_esc,
        1.0,
    )

    probabilities = _survival_analysis(normalized_event_matrix)

    return [_calculate_risk(events, probabilities) for events in event_matrix]


def _survival_function(event_matrix: NDArray[np.float64]) -> NDArray[np.float64]:
    return np.cumprod(np.exp(-event_matrix))


def _survival_analysis(event_matrix: NDArray[np.float64]) -> NDArray[np.float64]:
    survival_probabilities = _survival_function(event_matrix)
    return np.array([1.0, *survival_probabilities[:-1]], copy=False, dtype=np.float64)


def _calculate_risk(
    overlaps: NDArray[np.float64],
    probabilities: NDArray[np.float64],
) -> Probability:
    return Probability(float(np.sum(overlaps * probabilities)))
