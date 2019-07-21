#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Rearrangement Planning

    The Rearrangement Planning problem is concered with generating a sequence of
    pick-and-place actions to transform the initial configuration to the goal.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import pprint
import time

import pandas as pd

from rearrangement.planning.discretization import discretize
from rearrangement.planning import dlvhex


def generate_plan(
    continuous_configuration,
    collision_threshold=0.01,
    new=True,
    name="instance",
    verbose=False,
    accuracy=2,
    camera_distance=0,
):
    """
    Generates a plan to achieve the configuration.

    Parameters
    ----------
    continuous_configuration : Configuration
        The configuration, containing initial and goal poses of all bodies.
    collision_threshold : float
        In case rearrangement planning is neccessary, the minimium penetration
        depth between two bodies before a collision is declared.
    new : bool
        Whether or not new objects need to be planned for.
    name : str
        Base filename to use for debug files.
    verbose : bool
        Verbosity.
    accuracy : int
        Accuracy of poses by how many digits to be rounded to.

    Returns
    -------
    plan : DataFrame
        A dataframe describing the plan, ordered in the sequence of actions.

    """

    def merge_two_dicts(dict_1, dict_2):
        """
        Merges two dicts. From https://stackoverflow.com/questions/38987/.
        
        Parameters
        ----------
        dict_1 : dict
        dict_2 : dict

        Returns
        -------
        merged_dict : dict
        
        """

        merged_dict = dict_1.copy()
        merged_dict.update(dict_2)

        return merged_dict

    def place_news(continuous_configuration, accuracy):
        """
        Generates a placement plan for new bodies.

        Parameters
        ----------
        configuration : Configuration
            The configuration, containing initial and goal poses of all bodies.
        accuracy : int
            Accuracy of poses by how many digits to be rounded to.

        Returns
        -------
        data : list
            A list describing the plan, ordered in the sequence of actions.
        """

        data = []
        for body in continuous_configuration.news:
            from_pose = [round(x, accuracy) for x in body.init_pose]
            to_pose = [round(x, accuracy) for x in body.pose]
            data.append([body.name, None, from_pose, to_pose])
        return data

    columns = ["bodyName", "plannerID", "poseFrom", "poseTo"]

    # The logic here is if there are no orginal bodies, or if the original
    # bodies have not moved, and the plan does not involve bodies that are
    # non-uniform along the z-axis, we do not need a 'plan' at the task level,
    # we simply need to place new objects to their correct position.
    no_movement = not continuous_configuration.movement_info["status"]

    if no_movement and not new:
        # Create a plan of only placing new bodies to the surface.
        data = place_news(continuous_configuration, accuracy)

        time_elapsed = 0.0
    else:
        discretized_configuration = discretize(
            continuous_configuration=continuous_configuration,
            name=name,
            new=new,
            verbose=verbose,
        )

        # Query the hybrid planner for the minimum feasible plan.
        start_time = time.time()
        data = dlvhex.hybrid_plan(
            continuous_configuration=continuous_configuration,
            discretized_configuration=discretized_configuration,
            collision_threshold=collision_threshold,
            new=new,
            name=name,
            verbose=verbose,
            accuracy=accuracy,
        )
        time_elapsed = time.time() - start_time
        data = data + place_news(continuous_configuration, accuracy)

    plan = pd.DataFrame(data=data, columns=columns)

    hybrid_planning_res = {
        "number of rearranged objects": continuous_configuration.movement_info[
            "number"
        ],
        "number of plan steps": len(plan),
        "hybrid planning time": "{:.4f} s".format(time_elapsed),
    }

    if verbose:
        print ("Rearrangement Planning complete!")
        pprint.pprint(hybrid_planning_res)
        print ("\n")

    return plan
