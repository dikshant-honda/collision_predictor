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

if TYPE_CHECKING:
    from matplotlib.patches import Ellipse


def create_ellipse(
    position: tuple[float, float],
    sigma: tuple[float, float],
    angle: float,
    color: str = "green",
    alpha: float = 0.1,
    zorder: int = 0,
    edgecolor: str = "black",
    is_ego: bool = False,
) -> Ellipse:
    from matplotlib.patches import Ellipse

    width = 6.0 * sqrt(sigma[0])
    height = 6.0 * sqrt(sigma[1])

    facecolor = "none"
    if is_ego:
        facecolor = color
    return Ellipse(
        xy=position,
        width=width,
        height=height,
        angle=np.degrees(angle),
        alpha=alpha,
        facecolor=facecolor,
        edgecolor=color
    )
