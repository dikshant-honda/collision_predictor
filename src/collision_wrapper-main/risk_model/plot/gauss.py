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
# pylint: disable=import-outside-toplevel,too-many-locals
from __future__ import annotations

from risk_model import UncertainTrajectory

from .artist import create_ellipse
from .import_guard import ensure_matplotlib_pyplot


def plot_gaussians(trajectories: list[UncertainTrajectory], sample_rate: int = 10) -> None:
    """Plot trajectories with their uncertainties visualized as ellipses."""

    ensure_matplotlib_pyplot()
    from matplotlib.pyplot import (
        figure,
        gca,
        scatter,
        show,
        title,
        xlabel,
        xlim,
        ylabel,
        ylim,
        pause
    )

    figure()
    axes = gca()

    for trajectory in trajectories:
        if len(trajectory.positions) == 0:
            continue

        is_ego = trajectory.id == "ego"
        color = "green" if is_ego else "red"

        for position, sigma, angle in zip(
            trajectory.positions[::sample_rate],
            trajectory.covariance_matrices[::sample_rate],
            trajectory.angles()[::sample_rate],
        ):
            axes.add_artist(create_ellipse(position, (sigma[0][0], sigma[1][1]), angle, color=color))

        first_position = trajectory.positions[0]
        scatter(first_position[0], first_position[1], color="black", zorder=1)

    title("Gaussian sizes")
    xlim([-50.0, 250.0])
    ylim([-100.0, 200.0])
    ylabel("y-position in m")
    xlabel("x-position in m")
    show()
