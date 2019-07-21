#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    External Atoms

    Defines the external atoms to be used by the DLVHEX planner.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import dlvhex

from rearrangement.planning.dlvhex import utils, collision


def register():
    """Registers the external predicate into DLVHEX."""

    dlvhex.addAtom("isCollision", (dlvhex.PREDICATE,), 0)


def isCollision(location_predicate):
    """External predicate to detect if the plan is collision-free."""

    # Get all locations at all time steps
    dlvhex_input = [dlvhex.getValue(i) for i in dlvhex.getTrueInputAtoms()]

    # Get states from input
    plan = utils.get_task_plan(dlvhex_input)

    # Get information of continuous state and its discretization
    config_info, discrete_info = utils.get_dataframes(plan)
    verbose, col_thresh = utils.read_settings()

    # Check for collisions in the plan
    if not collision.colliding(
        plan,
        config_info,
        discrete_info,
        verbose=verbose,
        collision_threshold=col_thresh,
    ):
        return None

    return dlvhex.output(())
