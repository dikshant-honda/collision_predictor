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
from __future__ import annotations


class UnitInterval(float):
    """
    UnitInterval is a floating point value that is clipped between 0.0 and 1.0.

    Otherwise it is identical to a normal float.
    """

    def __new__(cls, value: float) -> UnitInterval:
        clipped_value = max(0.0, min(1.0, value))
        return float.__new__(cls, clipped_value)


Event = UnitInterval
Probability = UnitInterval
