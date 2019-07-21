#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    DLVHEX

    Manages DLVHEX-Python issues.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import ast

import pandas as pd

from rearrangement import TEMP_PATH
from rearrangement.planning.dlvhex import preprocessing, runner


def hybrid_plan(
    continuous_configuration,
    discretized_configuration,
    collision_threshold,
    new,
    name,
    verbose,
    accuracy,
):
    """
    Uses DLVHEX as a hybrid planner, integrating task and motion planning.

    PARAMETERS
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    discretized_configuration : Discrete
        The discretized representation of the configuration
    collision_threshold : float
        In case rearrangement planning is neccessary, the minimium penetration
        depth between two bodies before a collision is declared.
    new : bool
        Whether or not new objects need to be planned for.
    name : str
        Base filename to use for debug files.
    verbose : bool
        Verbosity.

    RETURNS
    -------
    data : list
        A list describing the plan, ordered in the sequence of actions.

    """

    def read_refined_task_plan(accuracy):
        """
        Reads the refined task plan output by the DLVHEX external atom.

        RETURNS
        -------
        data : list
            A list describing the plan, ordered in the sequence of actions.

        """

        def accurate(pose):
            return [round(x, accuracy) for x in pose]

        plan = pd.read_csv(TEMP_PATH + "/plan.csv")
        plan["poseFrom"] = plan["poseFrom"].apply(ast.literal_eval).apply(accurate)
        plan["poseTo"] = plan["poseTo"].apply(ast.literal_eval).apply(accurate)

        data = plan.values.tolist()

        return data

    # Store the information needed by the DLVHEX external atom(s) as CSV
    initial_info, goal_info, pandized_grid = preprocessing.preprocess(
        continuous_configuration, discretized_configuration
    )

    min_steps = continuous_configuration.movement_info["number"]

    # Perform hybrid planning using DLVHEX
    success = runner.run_dlvhex_planner(
        collision_threshold,
        initial_info,
        goal_info,
        pandized_grid,
        min_steps,
        name,
        verbose,
    )

    if success:
        refined_task_plan = read_refined_task_plan(accuracy)
        return refined_task_plan

    return []
