#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Execute risk model estimation given a biped json file
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

"""Execute risk model estimation given a biped json file."""

from __future__ import annotations

import json
import time
from argparse import ArgumentParser, Namespace
from copy import deepcopy
from typing import Any

from .risk_model.model import Position, Probability, Velocity
from predict_collisions import predict_collisions
from model import DataPoint

def _parse_arguments() -> Namespace:
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", help="Input file in biped json format", required=True)
    parser.add_argument("-o", "--output", help="Output file for storing risk prediction result")
    parser.add_argument(
        "-p", "--performance", type=int, metavar="RUNCOUNT", help="Measure average performance over RUNCOUNT runs"
    )
    return parser.parse_args()


def _load_input_data(filename: str) -> Any:
    """Loads data from json without other processing."""
    with open(filename, "r", encoding="utf-8") as file:
        return json.load(file)


def _save_results(results: dict[str, Any], filename: str) -> None:
    """Saves data json without other processing."""
    with open(filename, "w", encoding="utf-8") as file:
        json.dump(results, file, indent=4)


def _extract_data_points(
    sample_datas: list[dict[str, Any]],
) -> list[DataPoint]:
    return list(map(_data_point_from_json, sample_datas))


def _data_point_from_json(sample_data: dict[str, Any]) -> DataPoint:
    position = Position(sample_data["position"][0], sample_data["position"][1])
    if None not in sample_data["velocity"]:
        velocity = Velocity(sample_data["velocity"][0], sample_data["velocity"][1])
    else:
        velocity = Velocity(0, 0)

    return DataPoint(
        sample_data["tracking_id"],
        position=position,
        velocity=velocity,
        type=sample_data["label_name"] if "label_name" in sample_data else "unknown",
    )


def _annotate_input_data_with_risk(
    data: dict[str, Any],
    result: list[tuple[DataPoint, Probability]],
) -> dict[str, Any]:
    """Annotates data given in the original biped format with an additional risk element."""
    return_data = deepcopy(data)
    for data_point, probability in result:
        for element in return_data["data"]:
            if element["tracking_id"] == data_point.id:
                # element["risk"] = probability
                element["risk"] = max(probability)  # list of prob because of multiple vehicles
                break
    return return_data


def main() -> None:
    """Loads data from file, predicts risk, and optionally stores result."""
    args = _parse_arguments()

    input_data = _load_input_data(args.input)
    annotated_input_data = deepcopy(input_data)

    count = args.performance if not args.performance is None else 1
    total = 0.0

    for timestamp, sample in input_data.items():
        if len(sample["data"]) == 0:
            continue
        data_points = _extract_data_points(sample["data"])
        for _ in range(count):
            start = time.time()
            predicted_collisions = predict_collisions(data_points)
            end = time.time()
            total += end - start
            annotated_input_data[timestamp] = _annotate_input_data_with_risk(sample, predicted_collisions)

    if not args.performance is None:
        print(f"Mean calculation time: {total/count=}s")

    if not args.output is None:
        _save_results(annotated_input_data, args.output)


if __name__ == "__main__":
    main()
