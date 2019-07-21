#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Physics Utilities

    Functions that are useful for to the physics module in general.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import pybullet as p


def to_euler(quaternion):
    """
    Converts a unit quaternion to an Euler angle.

    Parameters
    ----------
    quaternion : list
        A list of 4 floats representing a quaternion in the [x, y, z, w] format.

    Returns
    -------
    euler : list
        A list of 3 floats representing the quaternion as euler angles in the
        [roll, pitch, yaw] format.
    """

    return p.getEulerFromQuaternion(quaternion)


def to_quaternion(euler):
    """
    Converts an Euler angle to a unit quaternion.

    Parameters
    ----------
    euler : list
        A list of 3 floats representing an euler angle in the [roll, pitch, yaw]
        format.

    Returns
    -------
    quaternion : list
        A list of 4 floats representing the euler angle as a unit quaternion in
        the [x, y, z, w] format.
    """

    return p.getQuaternionFromEuler(euler)
