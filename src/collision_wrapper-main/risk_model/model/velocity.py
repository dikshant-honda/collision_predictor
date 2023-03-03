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
from dataclasses import dataclass, field

import numpy as np

Radian = float


@dataclass(frozen=True)
class Velocity:
    x: float
    y: float

    speed: float = field(init=False)
    angle: Radian = field(init=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "speed", np.linalg.norm([self.x, self.y]))
        object.__setattr__(self, "angle", np.arctan2(self.y, self.x))
