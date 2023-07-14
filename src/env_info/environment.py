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
    """
    environment information class

    args:
        counter: the number of the traffic participants currently in the scenario
        vehicles: list of Traffic class type vehicles
        register: boolean for adding vehicle into the scenario
        deregister: boolean for removing vehicle out of the scenario
        interaction: boolean for checking the interaction with other traffic participants
    """
    def __init__(self, counter, vehicles, register, deregister, interaction) -> None:
        self.counter = counter
        self.vehicles = vehicles
        self.register = register
        self.deregister = deregister
        self.interaction = interaction