#! /usr/bin/env python3

from dataclasses import dataclass
import numpy as np
from env_info.vehicle_info import Traffic

@dataclass(frozen=True)
class Environment:
    counter: int
    vehicles: list(Traffic) 
    register: bool
    deregister: bool 
    interaction: bool

    def __init__(self, counter, vehicles, register, deregister, interaction) -> None:
        self.counter = counter
        self.vehicles = vehicles
        self.register = register
        self.deregister = deregister
        self.interaction = interaction