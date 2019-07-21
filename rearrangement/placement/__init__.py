#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Placement Generation

    The Placement Generation problem uses the configuration of the physics
    simulation as its state. It is a 3-layer deep nested problem of problems
    called Inner, Middle, and Outer. Each layer can be used, offering a level
    of control of the cost and state successor functions.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import json
import pprint
import time

from rearrangement import DEBUG_PATH
from rearrangement.search import LocalSearch
from rearrangement.physics import Configuration, Engine
from rearrangement.placement.inner import Inner
from rearrangement.placement.middle import Middle
from rearrangement.placement.outer import Outer
from rearrangement.placement.baselines import Random, RandomPotentialField


def generate_placement(
    query,
    algorithm="outer",
    collision_threshold=0.01,
    verbose=False,
    camera_distance=10.0,
    name="instance",
    engine=None,
):
    """
    Attempts returning a collision-free placement for the configuration.

    PARAMETERS
    ----------
    query : JSON
        The parsed query.
    algorithm : str
        Name of algorithm to use: 'outer', 'middle', etc.
    collision_threshold : float
        Penetration depth threshold for collision detection.
    verbose : bool
        Verbosity.
    camera_distance : float
        Distance of camera from center of surface.
    name : str
        Filename base to use for debug files.

    RETURNS
    -------
    solution : Configuration
        The (attempted to be) solved configuration.

    """

    if engine is None:
        engine = Engine()
        engine.connect(visual=verbose)
        engine.collision_threshold = collision_threshold
    config = engine.load_configuration(query)

    if verbose:
        img = engine.get_image(distance=camera_distance, new=False)
        img.save(DEBUG_PATH + "/{}-initial_placement.png".format(name))
        img = engine.get_image(distance=camera_distance)
        img.save(DEBUG_PATH + "/{}-random_placement.png".format(name))

    start_time = time.time()
    random_restart = True
    if algorithm.lower() == "random_sample":
        problem = Random(config, engine, start_time=start_time)

    elif algorithm.lower() == "inner":
        problem = Inner(config, engine, start_time=start_time)
        random_restart = False

    elif algorithm.lower() == "random_restart":
        problem = RandomPotentialField(config, engine, start_time=start_time)

    elif algorithm.lower() == "middle":
        problem = Middle(config, engine, start_time=start_time)

    elif algorithm.lower() == "outer":
        problem = Outer(config, engine, start_time=start_time)
    else:
        raise ValueError("Queried algorithm '{}' is unknown.".format(name))

    if verbose:
        solution, iterations = LocalSearch(
            problem, start_time=start_time, random_restart=random_restart
        ).simple(verbose=True)
    else:
        solution = LocalSearch(
            problem, start_time=start_time, random_restart=random_restart
        ).simple()

    col_info = engine.get_collision_info(solution)
    move_info = solution.movement_info

    no_collisions, penetration = col_info["number"], col_info["severity"]
    no_org_moved, org_movement = move_info["number"], move_info["severity"]
    time_elapsed = time.time() - start_time

    results = {
        "algorithm": algorithm,
        "cumulative original objects movement": "{:.4f} m".format(org_movement),
        "cumulative penetration depth": "{:.4f} m".format(penetration),
        "number of collisions": no_collisions,
        # "number of iterations": iterations,
        "number of original objects moved": no_org_moved,
        "placement time": "{:.4f} s".format(time_elapsed),
    }

    if verbose:
        print ("\n")
        print ("Placement Generation complete!")
        pprint.pprint(results)
        print ("\n")

        img = engine.get_image(distance=camera_distance)
        img.save(DEBUG_PATH + "/{}-goal_placement.png".format(name))

        with open(DEBUG_PATH + "/{}-goal_placement.json".format(name), "w") as my_ans:
            json.dump(config.dump_json(), my_ans)

    return solution
