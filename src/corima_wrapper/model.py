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


from typing import Literal, NamedTuple

from risk_model import Position, Velocity


class DataPoint(NamedTuple):
    id: str
    position: Position
    velocity: Velocity
    route: list
    type: Literal["person", "car", "bus", "unknown"] = "unknown"
