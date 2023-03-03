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
#

from __future__ import annotations

from typing import NamedTuple


class Tuple2D(NamedTuple):
    x: float
    y: float


class Position(Tuple2D):
    pass


class Sigma(Tuple2D):
    def add(self, other: Sigma) -> Sigma:
        return Sigma(
            x=self.x + other.x,
            y=self.y + other.y,
        )

    def clip(self, maximum: Sigma) -> Sigma:
        return Sigma(
            x=min(self.x, maximum.x),
            y=min(self.y, maximum.y),
        )
