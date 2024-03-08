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

from math import sqrt
from typing import TYPE_CHECKING
import numpy as np

from risk_model import Probability, UncertainTrajectory
from risk_model.matrix import rotate_matrix_2d

from .artist import create_ellipse
from .import_guard import ensure_matplotlib_pyplot

if TYPE_CHECKING:
    from matplotlib.artist import Artist
    from matplotlib.patches import Ellipse


def _update_ellipse(ellipse: Ellipse, trajectory: UncertainTrajectory, index: int) -> None:
    position = trajectory.positions[index]
    sigma = trajectory.covariance_matrices[index]
    angle = trajectory.angles()[index]
    sigma = rotate_matrix_2d(sigma, -angle)
    angle = np.degrees(angle)
    width =  3.0 * 2.0 * sqrt(sigma[0][0])
    height =  3.0 * 2.0 * sqrt(sigma[1][1])

    ellipse.angle = angle
    ellipse.center = position
    ellipse.width = width
    ellipse.height = height


def animate(ego_trajectory: UncertainTrajectory, other_trajectories: list[UncertainTrajectory]) -> None:

    ensure_matplotlib_pyplot()
    from matplotlib.animation import FuncAnimation
    from matplotlib.pyplot import figure, show, title, xlabel, xlim, ylabel, ylim, scatter, pause, clf

    fig = figure(14, figsize=(20, 20))
    axes = fig.gca()

    for agent_idx, trajectory in enumerate(other_trajectories):
        for traj_idx in range(len(trajectory.positions)):
            if traj_idx % 10 == 0:
                if len(trajectory.positions) == 0:
                    continue

                color = "red"
                zorder = 0

                ellipse = axes.add_artist(
                    create_ellipse(
                        (0, 0),
                        (0, 0),
                        0,
                        color=color,
                        alpha=1.0,
                        zorder=zorder,
                    )
                )
                _update_ellipse(ellipse, trajectory, traj_idx)
                scatter(trajectory.positions[0][0], trajectory.positions[0][1], color="blue", zorder=5)


    for traj_idx in range(len(ego_trajectory.positions)):
        if traj_idx % 5 == 0:
            if len(ego_trajectory.positions) == 0:
                continue
            is_ego = False      #True, true if we want to add color to the patch
            color = "green"
            zorder = 0

            ellipse = axes.add_artist(
                create_ellipse(
                    (0, 0),
                    (0, 0),
                    0,
                    color=color,
                    alpha=1.0,
                    zorder=zorder,
                    is_ego=is_ego
                )
            )
            _update_ellipse(ellipse, ego_trajectory, traj_idx)

    scatter(ego_trajectory.positions[0][0], ego_trajectory.positions[0][1], color="black", zorder=5)
    
    title("Gaussian sizes")
    xlim([-20.0, 20.0])
    ylim([-20.0, 20.0])
    ylabel("y-position in m")
    xlabel("x-position in m")

    show(block=False)
    pause(0.0001)
    clf()
