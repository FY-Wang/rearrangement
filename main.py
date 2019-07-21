#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Quickstart usage example.
    
    This script reads a json description of:
    1. a flat surface,
    2. the configuration of movable objects on the surface (originals),
    3. the configuration of immovable objects on the surface (obstacles),
    4. the objects to be added to the surface (news), and
    5. constraints on where each object should be [optional].

    This script then prints the the pick-and-place rearrangement plan.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0

    Repository: https://github.com/ardabbour/rearrangement/
"""

import argparse
import json
import pprint
import numpy as np

from rearrangement import DATA_PATH
from rearrangement import placement, planning


def main(
    query_path,
    algorithm,
    collision_threshold,
    random_seed,
    verbose,
    camera_distance,
    name,
):
    """
    Simple function to show minimal usage.

    PARAMETERS
    ----------
    query_path : str
        Path to the JSON query file.
    algorithm : str
        Name of algorithm to use: 'outer', 'middle', etc.
    collision_threshold : float
        Penetration depth threshold for collision detection.
    random_seed : int
        Seed for reporoducing randomness.
    verbose : bool
        Verbosity.
    camera_distance : float
        Distance of camera from center of surface.
    name : str
        Filename base to use for debug files.

    """

    # Use a seed for reproducibility
    np.random.seed(random_seed)

    # Extract the query
    query = json.load(open(query_path))

    # Placement Generation
    configuration = placement.generate_placement(
        query=query,
        algorithm=algorithm,
        collision_threshold=collision_threshold,
        name=name,
        verbose=verbose,
        camera_distance=camera_distance,
    )

    # Rearrangement Planning
    plan = planning.generate_plan(
        continuous_configuration=configuration,
        collision_threshold=collision_threshold,
        name=name,
        verbose=verbose,
        camera_distance=camera_distance,
        new=False,
    )

    # Show the resulting plan
    pprint.pprint(plan)


def str_to_bool(query_str):
    """
    Converts argument parsed as str to a bool.

    PARAMETERS
    ----------
    query_str : str
        Argument as str.

    RETURNS
    -------
    query_bool : bool
        Argument as bool.

    """

    if query_str.lower() in ["true", "t", "1"]:
        return True
    elif query_str.lower() in ["false", "f", "0"]:
        return False
    raise ValueError("Could not parse '{}' to bool type.".format(query_str))


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(
        description="Generate a goal placement and plan to achieve it."
    )
    PARSER.add_argument(
        "--query_path",
        "-q",
        help="JSON query file path",
        default=DATA_PATH + "/example-query.json",
        type=str,
    )
    PARSER.add_argument(
        "--verbose",
        "-v",
        help="Verbosity, outputs progress and info while solving, saves figs",
        default="False",
        type=str,
    )
    PARSER.add_argument(
        "--camera_distance",
        "-d",
        help="Distance from center for camera to capture figs (if verbose)",
        default=10.0,
        type=float,
    )
    PARSER.add_argument(
        "--name",
        "-n",
        help="Name of instance for labeling of plan output, figure, etc.",
        default="instance",
        type=str,
    )
    PARSER.add_argument(
        "--collision_threshold",
        "-c",
        help="Penetration depth threshold",
        default=0.01,
        type=float,
    )
    PARSER.add_argument(
        "--random_seed",
        "-r",
        help="Seed for reporoducing randomness.",
        default=None,
        type=int,
    )
    PARSER.add_argument(
        "--algorithm",
        "-a",
        help="Placement Generation algorithm",
        default="outer",
        type=str,
    )
    ARGS = PARSER.parse_args()

    main(
        query_path=ARGS.query_path,
        algorithm=ARGS.algorithm,
        collision_threshold=ARGS.collision_threshold,
        random_seed=ARGS.random_seed,
        verbose=str_to_bool(ARGS.verbose),
        camera_distance=ARGS.camera_distance,
        name=ARGS.name,
    )
