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

from typing import NamedTuple

from .types import Sigma


class UncertaintyConfig(NamedTuple):
    """Configuration for {Trajectory.uncertain}."""

    sigma_start: Sigma = Sigma(0.1, 0.1)
    sigma_max: Sigma = Sigma(0.4, 1.5)
    sigma_max_growth_factor: tuple[float, float] = (0.3, 0.3)
    sigma_max_growth_power: tuple[float, float] = (1.3, 0.9)
    speed_factor: tuple[float, float] = (0.03, 0.03)

    def _calc_factor(self, speed: float, min_speed: float = 1.0) -> Sigma:
        capped_speed = max(min_speed, speed)
        return Sigma(
            self.sigma_max_growth_factor[0] * pow(capped_speed, self.sigma_max_growth_power[0]),
            self.sigma_max_growth_factor[1] * pow(capped_speed, self.sigma_max_growth_power[1]),
        )

    def calc_speed_dependant_max_sigma(self, speed: float) -> Sigma:
        return self.sigma_start.add(self._calc_factor(speed)).clip(self.sigma_max)
