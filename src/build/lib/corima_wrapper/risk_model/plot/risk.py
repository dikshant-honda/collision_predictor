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
# pylint: disable=import-outside-toplevel
from __future__ import annotations

from typing import Callable, Literal

from risk_model import Probability

from .import_guard import ensure_matplotlib_pyplot

Scale = Literal["linear", "log", "symlog", "logit"]
Color = str
Id = str


def plot_risk(
    risks: list[tuple[Id, Probability]],
    scale: Scale = "linear",
    limit: tuple[float | None, float | None] = (None, None),
    color_changer: Callable[[Id], Color] = lambda _: "black",
) -> None:
    """Plot Risks of some kind in a basic plot."""

    ensure_matplotlib_pyplot()
    from matplotlib.pyplot import (
        figure,
        scatter,
        show,
        title,
        xlabel,
        ylabel,
        ylim,
        yscale,
        pause
    )

    figure()

    for risk_id, risk in risks:
        scatter(risk_id, risk, color=color_changer(risk_id))

    title("Risks")
    xlabel("timestamp")
    ylabel("risk")
    yscale(scale)
    ylim(limit)

    show()
