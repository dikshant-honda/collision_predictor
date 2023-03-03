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
# pylint: disable=import-outside-toplevel


def ensure_matplotlib_pyplot() -> None:
    """Internal helper function to raise an error if {matplotlib} is not installed."""

    try:
        import matplotlib.pyplot as _
    except ImportError as import_error:
        raise RuntimeError(
            "Cannot import matplotlib, did you install the library with the "
            "plotting extra? `pip install risk_model[plotting]`"
        ) from import_error
