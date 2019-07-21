#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Configuration Discretization

    The Configuration Discretization problem is that of finding the minimum
    number of non-uniform grid cells to represent a continuous configuration,
    such that each cell contains only one centroid of all the objects in the
    initial and tentative final configurations.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import time
import pprint

from rearrangement import DEBUG_PATH
from rearrangement.planning.discretization.discrete import Discrete


def discretize(continuous_configuration, name="instance", new=True, verbose=False):
    """
    Creates a discretized representation of the configuration.

    Parameters
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    name : str
        Base filename to use for debug files.
    new : bool
        Whether or not new objects need to be planned for.
    verbose : bool
        Verbosity.

    Returns
    -------
    disc_config : Discrete
        A minimal, discrete representation of the configuration.

    """

    start_time = time.time()
    disc_config = Discrete(continuous_configuration, new=new)

    new = new
    centroids = disc_config.centroids
    optimal_lines = disc_config.optimal_lines
    suboptimal_lines = disc_config.suboptimal_lines
    time_elapsed = time.time() - start_time

    results = {
        "new objects considered": new,
        "number of centroids": len(centroids),
        "number of suboptimal cells": (len(suboptimal_lines["hlines"]) - 1)
        * (len(suboptimal_lines["vlines"]) - 1),
        "number of optimal cells": (len(optimal_lines["hlines"]) - 1)
        * (len(optimal_lines["vlines"]) - 1),
        "discretization time": "{:.4f} s".format(time_elapsed),
    }

    if verbose:
        disc_config.plot_suboptimal(
            DEBUG_PATH + "/{}-discretized_configuration-suboptimal.pdf".format(name)
        )
        disc_config.plot_optimal(
            DEBUG_PATH + "/{}-discretized_configuration-optimal.pdf".format(name)
        )
        disc_config.plot_grid(
            DEBUG_PATH + "/{}-discretized_configuration-grid.pdf".format(name)
        )
        disc_config.plot_both(
            DEBUG_PATH + "/{}-discretized_configuration-both.pdf".format(name)
        )
        print ("Discrete Placement complete!")
        pprint.pprint(results)
        print ("\n")

    return disc_config
