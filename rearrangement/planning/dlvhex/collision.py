#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Collision

    Functions for the collision checking of the hybrid planner.

    Author: Abdul Rahman Dabbour
    Affiliation: CogRobo Lab, FENS, Sabanci University
    License: GNU Affero General Public License v3.0
    Repository: https://github.com/ardabbour/rearrangement/
"""

import copy
import json

import numpy as np

from rearrangement import DATA_PATH
from rearrangement.physics import Constraint, Engine
from rearrangement.planning.feasibility import Consistent, Plan
from rearrangement.search import LocalSearch


def colliding(
    plan,
    continuous_configuration_df,
    discrete_configuration_df,
    collision_threshold=0.01,
    verbose=False,
):
    """
    Checks to find if a plan is collision-free.

    PARAMETERS
    ----------
    continuous_configuration_df : DataFrame
        A dataframe describing the continuous configuration.
    discrete_configuration_df : DataFrame
        A dataframe describing the discretized configuration.
    collision_threshold : float
        The minimium penetration depth between two bodies before a collision is
        declared.
    verbose : bool
        Verbosity

    RETURNS
    -------
    is_colliding : bool
        Whether or not the plan contains a configuration in collision.

    """

    engine = Engine()
    engine.connect(visual=verbose)
    engine.collision_threshold = collision_threshold

    # Create constrained (according to DLVHEX cells) sequence of configs
    constrained_configs = get_constrained_configs(
        plan, continuous_configuration_df, discrete_configuration_df, engine
    )

    planner_ids = continuous_configuration_df[["oid", "name", "plannerID"]]

    # Get search state from queries
    state = Plan(constrained_configs, engine, planner_ids)

    # Initialize search problem
    problem = Consistent(state, engine)

    # If the initialized plan has no collisions, return False
    if problem.get_value(state)[0] == 0:
        state.save_plan(filename="plan", screenshots=verbose)
        engine.disconnect()
        return False

    # Else, search for solution
    solution = LocalSearch(problem, timeout=2, random_restart=False).simple()

    # If the plan cannot be freed from collisions, return True
    if problem.get_value(solution)[0] > 0:
        engine.disconnect()
        return True

    # Else, store the plan with continuous poses in a csv and return False
    solution.save_plan(filename="plan", screenshots=verbose)

    engine.disconnect()
    return False


def get_constrained_configs(plan, cont_config_df, disc_conf_df, engine):
    """Returns a list of configurations, each with its appropriate constraints."""

    configs = []
    init_query = create_query(cont_config_df, disc_conf_df, plan[0])
    init_config = engine.load_configuration(json.loads(init_query))
    configs.append(copy.deepcopy(init_config))

    engine.configuration = copy.deepcopy(init_config)
    this_config = copy.deepcopy(init_config)

    for step in plan[1:]:
        next_query = create_query(cont_config_df, disc_conf_df, step)
        next_config = copy.deepcopy(this_config)
        for phys_body in next_config.movable:
            name = phys_body.name
            json_constraints = [
                item
                for _, item in json.loads(next_query)["originals"][name][
                    "constraints"
                ].items()
            ]
            body_constraints = []
            for x in json_constraints:
                body_constraints.append(Constraint(x["shape"], x["geometry"]))
            phys_body.constraints = body_constraints
            pose = json.loads(next_query)["originals"][name]["pose"]
            phys_body.pose = pose

        configs.append(next_config)

    for config in configs:
        engine.push_bodies(config)

    return configs


def create_query(cont_config_df, disc_conf_df, step):
    """
    Creates a JSON query for that can be loaded as a confuguration.

    PARAMETERS
    ----------
    continuous_configuration_df : DataFrame
        A dataframe describing the continuous configuration.
    discrete_configuration_df : DataFrame
        A dataframe describing the discretized configuration.
    step : int
        The step of the plan for which the query is to be created.

    RETURNS
    -------
    query : JSON
        A constrained configuration formatted as a query.

    """

    def get_cell(cell_no):
        """Returns the cell coordinates given the cell number."""
        cell_row = disc_conf_df[disc_conf_df["cell"] == cell_no].squeeze()
        return cell_row["coordinates"]

    def in_init(body):
        """Checks if the body is in its initial destination."""

        if body["initCell"] == step[step["plannerID"] == body.name]["cell"].values[0]:
            return True
        return False

    def in_goal(body):
        """Checks if the body is in its goal destination."""

        if body["goalCell"] == step[step["plannerID"] == body.name]["cell"].values[0]:
            return True
        return False

    def get_random_pose(cell_no):
        """Defines a random pose depending within the boundaries of the body's
        containing cell."""

        [left, upper, right, lower] = get_cell(cell_no)

        pose_x = np.random.uniform(left, right)
        pose_y = np.random.uniform(lower, upper)
        pose_theta = np.random.uniform(0, 2 * np.pi)

        pose = [pose_x, pose_y, pose_theta]

        return pose

    def get_center_pose(cell_no):
        """Defines a random pose depending within the boundaries of the body's
        containing cell."""

        [left, upper, right, lower] = get_cell(cell_no)

        pose_x = ((right - left) / 2) + left
        pose_y = ((upper - lower) / 2) + lower
        pose_theta = np.pi

        pose = [pose_x, pose_y, pose_theta]

        return pose

    def pack(bodies, random=False):
        """Packs the set of bodies into a dictionary."""

        dic = {}
        for _, body in bodies.iterrows():
            if in_init(body):
                pose = body["initPose"]
                constraints = {
                    "trans_const": {
                        "shape": "circular",
                        "geometry": {"center": pose[:-1], "radius": 0},
                    },
                    "rot_const": {
                        "shape": "rotational",
                        "geometry": {"max": pose[-1], "min": pose[-1]},
                    },
                }
            elif in_goal(body):
                pose = body["goalPose"]
                constraints = {
                    "trans_const": {
                        "shape": "circular",
                        "geometry": {"center": pose[:-1], "radius": 0},
                    },
                    "rot_const": {
                        "shape": "rotational",
                        "geometry": {"max": pose[-1], "min": pose[-1]},
                    },
                }
            else:
                cell_no = step[step["plannerID"] == body.plannerID].squeeze()["cell"]

                if random:
                    pose = get_random_pose(cell_no)
                else:
                    pose = get_center_pose(cell_no)
                [left, upper, right, lower] = get_cell(cell_no)
                constraints = {
                    "trans_const": {
                        "shape": "rectangular",
                        "geometry": {
                            "min x": left,
                            "max x": right,
                            "min y": lower,
                            "max y": upper,
                        },
                    }
                }
            path = body["path"]
            z_off = 0.5
            key = body["name"]
            value = {
                "path": path,
                "pose": pose,
                "z offset": z_off,
                "area": 1.0,
                "constraints": constraints,
            }
            dic[key] = value

        return dic

    surface = {
        "path": DATA_PATH + "/simple/models/surface.urdf",
        "pose": [0.0, 0.0, 0.0],
        "z offset": 0.5,
        "area": 50.0,
    }

    obss = cont_config_df[cont_config_df["category"] == "obs"]
    orgs = cont_config_df[cont_config_df["category"] == "org"]
    obstacles = pack(obss)
    originals = pack(orgs)

    data = {
        "surface": surface,
        "obstacles": obstacles,
        "originals": originals,
        "news": {},
    }

    return json.dumps(data, sort_keys=True, indent=4)
