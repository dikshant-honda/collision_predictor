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

from operator import itemgetter

from risk_model import Trajectory, UncertainTrajectory, compute_survival
from risk_model.event_calculators.overlap_calculator import calculate_overlaps

from .model import DataPoint


def get_safest_trajectory(
    ego_trajectories: list[UncertainTrajectory], datapoints: list[DataPoint]
) -> UncertainTrajectory:
    """
    Compound function to calculate the 'safest' trajectory.

    Straight up taken from the examples of the risk-model library.
    """

    risk_list: list[tuple[UncertainTrajectory, float]] = []

    for ego_trajectory in ego_trajectories:
        delta_t = ego_trajectory.delta_t
        trajectory_length = len(ego_trajectory.positions)
        uncertain_trajectories = [
            Trajectory.linear_prediction(
                trajectory_id=datapoint.id,
                position=datapoint.position,
                velocity=datapoint.velocity,
                delta_t=delta_t,
                trajectory_length=trajectory_length,
            ).uncertain()
            for datapoint in datapoints
        ]

        events = calculate_overlaps(ego_trajectory, uncertain_trajectories)

        max_risk = max(compute_survival(events, delta_t=delta_t))

        risk_list.append((ego_trajectory, max_risk))

    return min(risk_list, key=itemgetter(1))[0]
