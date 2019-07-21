#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Physics

    This module contains basic classes, methods, and functions of the physics
    simulation (using Bullet as a backend for pybullet) for the GRMMO problem.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""


from rearrangement.physics.utils import to_euler, to_quaternion
from rearrangement.physics.constraint import Constraint
from rearrangement.physics.body import Body
from rearrangement.physics.configuration import Configuration
from rearrangement.physics.engine import Engine
