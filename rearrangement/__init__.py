#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Placement and Planning for Rearrangement Problems

    Code repository for Dabbour et. al [https://arxiv.org/abs/1906.08494].
    Abstract:

    "For planning rearrangements of objects in a clutter, it is required to know
    the goal configuration of the objects. However, in real life scenarios, this
    information is not available most of the time. We introduce a novel method
    that computes a collision-free placement of objects on a cluttered surface,
    while minimizing the total number and amount of displacements of the
    existing moveable objects. Our method applies nested local searches that
    perform multi-objective optimizations guided by heuristics. Experimental
    evaluations demonstrate high computational efficiency and success rate of
    our method, as well as good quality of solutions."

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

from os.path import dirname, abspath, isdir
from os import makedirs

LIB_PATH = abspath(dirname(__file__))
DATA_PATH = abspath(LIB_PATH + "/data")
TEMP_PATH = abspath(LIB_PATH + "/temp")
DEBUG_PATH = abspath(LIB_PATH + "/debug")

for path in [TEMP_PATH, DEBUG_PATH]:
    if not isdir(path):
        makedirs(path)
