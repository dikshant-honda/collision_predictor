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


from enum import Enum

from .risk_model import Sigma, UncertaintyConfig


class ObjectType(Enum):
    CAR = "car"
    PEDESTRIAN = "pedestrian"
    TRUCK = "truck"
    BICYCLE = "bicycle"
    MOTORCYCLE = "motorcycle"


UNCERTAINTY_CONFIG = {
    ObjectType.CAR: UncertaintyConfig(
        sigma_start=Sigma(2.25 / 3.0, 1.0 / 3.0),
        sigma_max=Sigma(45.0 / 3.0, 1.5 / 3.0),
        sigma_max_growth_factor=(0.3, 0.3),
        sigma_max_growth_power=(1.12, 1.12),
        speed_factor=(0.125, 0.1),
    ),
    ObjectType.PEDESTRIAN: UncertaintyConfig(
        sigma_start=Sigma(0.1, 0.1),
        sigma_max=Sigma(0.4, 1.5),
        sigma_max_growth_factor=(0.3, 0.3),
        sigma_max_growth_power=(1.3, 0.9),
        speed_factor=(0.03, 0.03),
    ),
    ObjectType.TRUCK: UncertaintyConfig(
        sigma_start=Sigma(5.0 / 3.0, 1.5 / 3.0),
        sigma_max=Sigma(35 / 3.0, 2.0 / 3.0),
        sigma_max_growth_factor=(0.3, 0.3),
        sigma_max_growth_power=(1.12, 1.12),
        speed_factor=(0.125, 0.1),
    ),
    ObjectType.BICYCLE: UncertaintyConfig(
        sigma_start=Sigma(1.0 / 3.0, 0.4 / 3.0),
        sigma_max=Sigma(10 / 3.0, 1.0 / 3.0),
        sigma_max_growth_factor=(0.3, 0.3),
        sigma_max_growth_power=(1.12, 1.12),
        speed_factor=(0.125, 0.1),
    ),
    ObjectType.MOTORCYCLE: UncertaintyConfig(
        sigma_start=Sigma(1.25 / 3.0, 0.6 / 3.0),
        sigma_max=Sigma(45 / 3.0, 1.0 / 3.0),
        sigma_max_growth_factor=(0.3, 0.3),
        sigma_max_growth_power=(1.12, 1.12),
        speed_factor=(0.125, 0.1),
    ),
}
