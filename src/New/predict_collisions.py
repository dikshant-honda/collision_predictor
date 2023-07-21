#! /usr/bin/env python3

import numpy as np

from IDM.idm import IDM, predict_trajectory
from New.circular_noise import add_noise
from New.circular_overlap import overlap


if __name__ == "__main__":

    time_horizon = 50
    time_step = 0.1

    # initializations
    ego_position = 0
    ego_speed = 15

    lead_position = 50
    lead_speed = 10

    # calling the IDM class object
    idm = IDM()


    result = predict_trajectory(
        idm, ego_position, ego_speed, lead_position, lead_speed, time_horizon, time_step)

    time, ego_trajectory, lead_trajectory = result

