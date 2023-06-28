#! /usr/bin/env python3

class Environment:
    def __init__(self, counter, vehicles, register, deregister, interaction) -> None:
        self.counter = counter
        self.vehicles = []
        for i in vehicles:
            self.vehicles.append(i)
        self.register = register
        self.deregister = deregister
        self.interaction = interaction