#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Error handling

    Handling of errors

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""


def type_error(var_name, valid_type, invalid_type):
    """Returns a TypeError for the specific case.

    Keyword arguments:
    var_name -- name of the variable raising the error (str)
    valid_type -- type of the variable that should hold (type)
    invalid_type -- type of the vairable
    """

    return TypeError(
        "'{}' type used for '{}'. Only '{}' type is valid for '{}'.".format(
            invalid_type.__name__, var_name, valid_type.__name__, var_name
        )
    )


def value_error(var_name, condition, value):

    return ValueError(
        "'{}' has value of '{}'. Only '{}' is valid for '{}'.".format(
            var_name, value, condition, var_name
        )
    )


def length_error(var_name, valid_length, invalid_length):

    return value_error("Length of {}".format(var_name), valid_length, invalid_length)
